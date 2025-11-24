Drone-based Search and Rescue Object Detection System

산악 및 야외 환경에서 드론을 활용하여 실종자를 실시간으로 탐색하기 위한 객체 탐지 시스템
단일 모델의 한계를 보완하기 위해, 서로 다른 데이터셋으로 학습된 두 개의 YOLO 모델을 앙상블하여 Recall을 극대화

Model Architecture

시스템은 YOLOv8 Medium 모델을 베이스로 사용하며, 각기 다른 데이터셋으로 학습된 두 모델을 사용
Model A / YOLOv8m / VisDrone (Human subset) / 상공의 소형 객체 탐지 특화. 배경 노이즈 오탐 가능성 존재
Model B / YOLOv8m | SARD (SAR Dataset) | Pose Specialist | 숲속이나 야외 환경의 실종자 탐지 특화

Model Details

1.  Model A: VisDrone Trained 
    드론 항공 촬영 데이터셋인 VisDrone에서 사람(Pedestrian, People) 클래스만 추출하여 학습

Model Specs

  - Architecture: YOLOv8 Medium (Ultralytics)
  - Parameters: 25.9M
  - Input Size: 640x640
  - Dataset: VisDrone 2019 (Human subset)

Training Configuration

  - Epochs: 200
  - Loss Function: Default YOLOv8 Loss
  - Performance: 상공에서의 작은 객체 탐지
  - Limitation: 바위나 나무 등을 사람으로 오인하는 오탐(FP)이 존재
 
2.  Model B: SARD Fine-tuned
    실종자 수색 전용 데이터셋(SARD)을 사용하여, 숲속이나 야외 환경의 실종자 탐지 특화하여 학습한 모델

Model Specs

  - Architecture: YOLOv8 Medium (Ultralytics)
  - Base Weights: yolov8m.pt (COCO Pre-trained)
  - Parameters: 25.9M
  - Dataset: SARD (Search and Rescue Dataset)

Training Configuration (Fine-tuning)
기존 yolov8m 모델의 특징 추출 능력을 보존하기 위해 Backbone Freeze 적용

  - Epochs: 100
  - Batch Size: 16
  - Frozen Layers: Backbone 10 layers (freeze=10)
  - Hyperparameters:
      - lr0: 0.001 (Low Learning Rate for stability)
      - iou: 0.5
      - imgsz: 640

Training Code Snippet

python
model = YOLO('yolov8m.pt')
results = model.train(
    data='sard.yaml',
    epochs=100,
    imgsz=640,
    freeze=10,  # Prevent catastrophic forgetting
    project='sard_finetune',
    lr0=0.001
)


Model Selection and Optimization
실시간 처리 속도와 정확도의 균형을 위해 YOLOv8 Medium 모델로 설정 NVIDIA TensorRT(.engine)로 변환하여 RTX 4070 Super 기준 60 FPS 이상의 실시간 성능 확보하여 사용, .pt 파일을 .engine파일로 변환하여 사용하는 것 추천
NVIDIA 그래픽 카드를 사용하지 않는 경우 기존 PyTorch 모델(.pt) 사용 가능
하드웨어 사양이 부족한 경우 더 가벼운 모델인 YOLOv8s를 기반으로 훈련한 모델을 사용하거나(small_model 폴더), 프레임 처리 단위를 조절하는 프레임 스킵 기능을 사용할 수 있음

Video Input System
OpenIPC를 통해 드론 영상을 실시간으로 수신 가능하며, HDMI 캡쳐보드 또는 OBS 가상 카메라를 사용하여 영상 처리 가능

Ensemble Strategy 

각 모델의 장점을 취하고 오탐을 억제하는 전략을 사용함. 각 모델의 confidence는 조절가능(현재 SARD : 0.15, VisDrone : 0.35)

1.  Size-based Filtering
    VisDrone 모델의 배경 오탐 문제를 해결하기 위해 크기 기반 필터링 적용

  - Model B (SARD): 모든 크기의 객체를 탐지하며 가중치 2 부여
  - Model A (VisDrone): 화면의 n% 미만인 객체만 탐지하도록 제한 (현재 5% 미만. 5~10% 사이 추천)

2.  Weighted Boxes Fusion (WBF)
    두 모델이 찾아낸 박스들을 가중 평균 방식으로 융합

  - 서로 겹치는 박스는 좌표를 보정하여 정확도 향상
  - 한 모델만 찾아낸 박스도 최종 결과에 포함하여 미탐지 최소화 및 Recall 극대화 목표

Key Features

1.  Dual Mode Support

  - Local Analysis: 저장된 동영상 파일(.mp4) 분석.
  - Real-time Stream: OBS 가상 카메라 및 HDMI 캡쳐보드를 통한 실시간 드론 영상 분석

2.  Real-time Alert System

  - 설정된 신뢰도 이상의 객체 발견 시 경고음 출력 및 화면 알림 (현재는 미사용으로 체크)

3.  Performance Optimization

  - TensorRT 가속 적용.
  - Frame Skipping 옵션 제공 (배터리 절약(labtop) / 저사양 PC 대응)

Environment and Requirements

  - OS: Windows 10/11
  - GPU: NVIDIA GeForce RTX 3060 or higher (RTX 4070 Super Recommended) 
  - Language: Python 3.8+
  - Libraries:
      - ultralytics (YOLOv8)
      - ensemble-boxes (WBF)
      - opencv-python
      - tensorrt (Optional)

How to Run

1.  Install dependencies

pip install ultralytics ensemble-boxes opencv-python

2.  Run the main system

python detection.py

License

This project uses datasets and models based on:

  - SARD Dataset: MIT License
      - Source: [https://www.kaggle.com/datasets/nikolasgegenava/sard-search-and-rescue](https://www.kaggle.com/datasets/nikolasgegenava/sard-search-and-rescue)
  - VisDrone Dataset: Academic Use Only
  - Ultralytics YOLO: AGPL-3.0
