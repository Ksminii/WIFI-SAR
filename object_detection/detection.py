import cv2
import numpy as np
import os
import time
import threading
import platform
from datetime import datetime
from ultralytics import YOLO
from ensemble_boxes import weighted_boxes_fusion

# 윈도우 알림음 사용을 위한 라이브러리
try:
    import winsound
except ImportError:
    winsound = None 

# ==========================================
# [사용자 설정 파라미터]
# ==========================================
# 모델 경로
MODEL_SARD_PATH = 'Object_detection/yolo_sard_finetuning.engine'  # SARD
MODEL_VIS_PATH  = 'Object_detection/visdrone.engine'              # VisDrone

DEFAULT_VIDEO_PATH = "test1.mp4"

# 앙상블 설정
CONF_SARD = 0.15     # SARD Confidence
CONF_VIS  = 0.35     # VisDrone Confidence
VIS_AREA_THR = 0.05  # VisDrone 크기 필터 (화면의 5%)

# 알림 설정
ALERT_THRESHOLD = 1.0  # CONFIDENCE 기준 이상의 사람이 발견되면 소리 울림 *미사용
ALERT_COOLDOWN = 2.0    # 알림 간격

# 결과 저장 경로
OUTPUT_DIR = "sar_results"
# ==========================================

class SoundAlert:
    #알림음을 백그라운드로 재생
    def __init__(self):
        self.last_alert_time = 0

    def trigger(self):
        current_time = time.time()
        if current_time - self.last_alert_time > ALERT_COOLDOWN:
            self.last_alert_time = current_time
            threading.Thread(target=self._play_sound, daemon=True).start()

    def _play_sound(self):
        try:
            if winsound:
                winsound.Beep(1000, 300)
            else:
                print('\a')
        except:
            pass

class EnsembleDetector:
    def __init__(self):
        print(f"Model loading")
        self.model_sard = YOLO(MODEL_SARD_PATH, task='detect')
        self.model_vis = YOLO(MODEL_VIS_PATH, task='detect')
        print("Model ready")

    def run_ensemble(self, img):
        h, w, _ = img.shape
        box_list, score_list, label_list, weights = [], [], [], []

        # SARD 
        res_sard = self.model_sard.predict(img, conf=CONF_SARD, verbose=False)[0]
        if len(res_sard.boxes) > 0:
            box_list.append(res_sard.boxes.xyxyn.cpu().numpy().tolist())
            score_list.append(res_sard.boxes.conf.cpu().numpy().tolist())
            label_list.append([0] * len(res_sard.boxes))
            weights.append(2) # 가중치 2

        # VisDrone 
        res_vis = self.model_vis.predict(img, conf=CONF_VIS, verbose=False)[0]
        if len(res_vis.boxes) > 0:
            vis_b, vis_s, vis_l = [], [], []
            raw_boxes = res_vis.boxes.xyxyn.cpu().numpy()
            raw_scores = res_vis.boxes.conf.cpu().numpy()
            found = False
            for box, score in zip(raw_boxes, raw_scores):
                area = (box[2] - box[0]) * (box[3] - box[1])
                if area < VIS_AREA_THR: # 크기 필터링
                    vis_b.append(box.tolist())
                    vis_s.append(score)
                    vis_l.append(0)
                    found = True
            if found:
                box_list.append(vis_b)
                score_list.append(vis_s)
                label_list.append(vis_l)
                weights.append(1) # 가중치 1

        if not box_list: return []

        # [3] WBF 융합
        boxes, scores, labels = weighted_boxes_fusion(
            box_list, score_list, label_list, weights=weights, iou_thr=0.55, skip_box_thr=0.15
        )

        final_dets = []
        for box, score in zip(boxes, scores):
            x1, y1, x2, y2 = int(box[0]*w), int(box[1]*h), int(box[2]*w), int(box[3]*h)
            final_dets.append([x1, y1, x2, y2, score])

        return final_dets

def process_video(source, is_live=False, skip_frames=0):
    detector = EnsembleDetector()
    alerter = SoundAlert()
    
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_path = os.path.join(OUTPUT_DIR, f"result_{timestamp}.mp4")

    # 비디오 소스 열기
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"❌ Failed to open video source: {source}")
        return

    if is_live:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    # 비디오 정보 가져오기
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps    = cap.get(cv2.CAP_PROP_FPS)
    if fps == 0: fps = 30 

    out = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))
    
    print(f"\nStarting Analysis (Frame Skip: {skip_frames})")
    print(f"Saving result to: {save_path}")
    print("Press 'q' in the video window to quit.")

    frame_idx = 0
    prev_time = 0
    last_detections = []

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret: break

        frame_idx += 1

        # [프레임 스킵]
        # skip_frames=0 이면 매 프레임 실행
        # skip_frames=2 이면 3프레임마다 1번 실행
        if (frame_idx - 1) % (skip_frames + 1) == 0:
            last_detections = detector.run_ensemble(frame)
            
            # [알림]
            # 탐지된 것 중 가장 높은 점수가 기준을 넘으면 소리 발생
            if last_detections:
                max_score = max([d[4] for d in last_detections])
                if max_score >= ALERT_THRESHOLD:
                    alerter.trigger()
                    cv2.rectangle(frame, (0, 0), (width, 30), (0, 0, 255), -1)
                    cv2.putText(frame, f"ALERT! FOUND ({max_score:.2f})", (10, 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # 시각화
        for det in last_detections:
            x1, y1, x2, y2, score = det
            
            # 알림 기준 넘는 건 빨간색, 아니면 초록색으로 표기
            color = (0, 0, 255) if score >= ALERT_THRESHOLD else (0, 255, 0)
            
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"Human {score:.2f}", (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # 상태 표시
        curr_time = time.time()
        curr_fps = 1 / (curr_time - prev_time) if prev_time > 0 else 0
        prev_time = curr_time

        status_text = f"Live FPS: {curr_fps:.1f} | Skip: {skip_frames}"
        cv2.putText(frame, status_text, (10, height - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        # 결과 저장 및 출력
        out.write(frame)
        cv2.imshow('SAR Operation System', frame)

        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    out.release()
    cv2.destroyAllWindows()
    print("\nfinish.")

def main():
    print("="*40)
    print(" Object Detection - SAR Rescue Ensemble System ")
    print("="*40)
    print("select mode:")
    print("1. local video/Image file analysis")
    print("2. live video analysis")
    
    choice = input("\nSelect (1 or 2): ").strip()

    # 프레임 스킵 설정
    try:
        skip_input = input("Frame Skip select (0=Skip X/Recommended, 2=faster 3x): ").strip()
        skip_frames = int(skip_input) if skip_input else 0
    except:
        skip_frames = 0

    if choice == '1':
        #local video file
        print(f"\nDefault Video Path: {DEFAULT_VIDEO_PATH}")
        path_input = input("Enter video path (Press Enter to use default): ").strip().replace('"', '')
        
        # 기본값 사용
        if path_input == "":
            path = DEFAULT_VIDEO_PATH
        else:
            path = path_input

        if os.path.exists(path):
            process_video(path, is_live=False, skip_frames=skip_frames)
        else:
            print(f"❌ File not found: {path}")

    elif choice == '2':
        print("\n[Connect Live Camera Source]")
        
        idx_input = input("Enter Camera Index (Default 0): ").strip()
        camera_idx = int(idx_input) if idx_input.isdigit() else 0
        
        process_video(camera_idx, is_live=True, skip_frames=skip_frames)

    else:
        print("Invalid selection.")

if __name__ == '__main__':
    # Prevent multiprocessing conflict
    try:
        import multiprocessing
        multiprocessing.freeze_support()
    except: pass
    
    main()