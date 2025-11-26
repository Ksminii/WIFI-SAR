# Drone-based Search and Rescue Object Detection System

산악 및 야외 환경에서 드론을 활용하여 실종자를 실시간으로 탐색하기 위한 객체 탐지 시스템

단일 모델의 한계를 보완하기 위해, 서로 다른 데이터셋으로 학습된 두 개의 YOLO 모델을 앙상블하여 Recall을 극대화

## Model Architecture

시스템은 YOLOv8 Medium 모델을 베이스로 사용하며, 각기 다른 데이터셋으로 학습된 두 모델을 사용

| Model | Architecture | Dataset | Specialization |
|-------|-------------|---------|----------------|
| Model A | YOLOv8m | VisDrone (Human subset) | 상공의 소형 객체 탐지 특화. 배경 노이즈 오탐 가능성 존재 |
| Model B | YOLOv8m | SARD (SAR Dataset) | 숲속이나 야외 환경의 실종자 탐지 특화 |

### Training Methodology 
데이터셋의 크기와 도메인 특성에 맞춰 차별화된 전이 학습 전략을 적용

- Model A (VisDrone): Full Fine-tuning
    - 이유: 데이터셋의 크기가 크고, 지상 뷰(COCO)와 드론 뷰(VisDrone) 간의 도메인 차이가 큼
    - 방법: 모델의 모든 레이어 파라미터를 업데이트하여 새로운 도메인에 대한 적응력을 극대화

- Model B (SARD): Backbone Freezing (Partial Fine-tuning)
    - 이유: 데이터셋의 크기가 상대적으로 작아 과적합 위험이 있으며, 탐지 대상의 특징이 사전 학습된 모델의 지식과 유사함.
    - 방법: Backbone를 Freeze하여 기존의 강력한 시각적 특징 추출 능력을 보존하고, Classifier 부분만 집중적으로 재학습하여 효율성을 높임.

## Model Details

### 1. Model A : VisDrone Trained

드론 항공 촬영 데이터셋인 VisDrone에서 사람(Pedestrian, People) 클래스만 추출하여 학습

#### Model Specs
- Architecture: YOLOv8 Medium (Ultralytics)
- Parameters: 25.9M
- Input Size: 640x640
- Dataset: VisDrone 2019 (Human subset)

#### Training Configuration
- Epochs: 200
- Loss Function: Default YOLOv8 Loss
- Performance: 상공에서의 작은 객체 탐지
- Limitation: 바위나 나무 등을 사람으로 오인하는 오탐(FP)이 존재

### 2. Model B : SARD Fine-tuned

실종자 수색 전용 데이터셋(SARD)을 사용하여, 숲속이나 야외 환경의 실종자 탐지 특화하여 학습한 모델

#### Model Specs
- Architecture: YOLOv8 Medium (Ultralytics)
- Base Weights: yolov8m.pt (COCO Pre-trained)
- Parameters: 25.9M
- Dataset: SARD (Search and Rescue Dataset)

#### Training Configuration (Fine-tuning)

기존 yolov8m 모델의 특징 추출 능력을 보존하기 위해 Backbone Freeze 적용

- Epochs: 100
- Batch Size: 16
- Frozen Layers: Backbone 10 layers (freeze=10)
- Hyperparameters:
  - `lr0`: 0.001 (Low Learning Rate for stability)
  - `iou`: 0.5
  - `imgsz`: 640

#### Training Code Snippet

```python
model = YOLO('yolov8m.pt')
results = model.train(
    data='sard.yaml',
    epochs=100,
    imgsz=640,
    freeze=10,  # Prevent catastrophic forgetting
    project='sard_finetune',
    lr0=0.001
)
```

## Model Selection and Optimization

실시간 처리 속도와 정확도의 균형을 위해 YOLOv8 Medium 모델로 설정

- TensorRT 최적화: NVIDIA TensorRT(.engine)로 변환하여 RTX 4070 Super 기준 60 FPS 이상의 실시간 성능 확인. `.pt` 파일을 `.engine` 파일로 변환하여 사용하는 것 추천
- NVIDIA 미사용 시: 기존 PyTorch 모델(.pt) 사용 가능
- 저사양 PC 대응:
  - 더 가벼운 모델인 YOLOv8s를 기반으로 훈련한 모델 사용 (`small_model` 폴더)
  - 프레임 처리 단위를 조절하는 프레임 스킵 기능 사용 가능

## Video Input System

- OpenIPC: 드론 영상을 실시간으로 수신 가능
- HDMI 캡쳐보드: 영상 처리 가능
- OBS 가상 카메라: 영상 처리 가능

## Ensemble Strategy

각 모델의 장점을 취하고 오탐을 억제하는 전략을 사용

각 모델의 confidence는 조절 가능 (현재 SARD: 0.15, VisDrone: 0.35)

### 1. Size-based Filtering

VisDrone 모델의 배경 오탐 문제를 해결하기 위해 크기 기반 필터링 적용

- Model B (SARD): 모든 크기의 객체를 탐지하며 가중치 2 부여
- Model A (VisDrone): 화면의 n% 미만인 객체만 탐지하도록 제한 (현재 5% 미만. 5~10% 사이 추천)

### 2. Weighted Boxes Fusion (WBF)

두 모델이 찾아낸 박스들을 가중 평균 방식으로 융합

- 서로 겹치는 박스는 좌표를 보정하여 정확도 향상
- 한 모델만 찾아낸 박스도 최종 결과에 포함하여 미탐지 최소화 및 Recall 극대화 목표

## Key Features

### 1. Dual Mode Support
- Local Analysis: 저장된 동영상 파일(.mp4) 분석
- Real-time Stream: OBS 가상 카메라 및 HDMI 캡쳐보드를 통한 실시간 드론 영상 분석

### 2. Real-time Alert System
- 설정된 신뢰도 이상의 객체 발견 시 경고음 출력 및 화면 알림 (현재는 1.0으로 설정)

### 3. Performance Optimization
- Frame Skipping 옵션 제공 (배터리 절약(laptop) / 저사양 PC 대응)
- lightweight Option 제공('yolov8s' 기반의 경량 모델(small_model) 지원)

## Environment and Requirements

- OS: Windows 10/11
- GPU: NVIDIA GeForce RTX 3060 or higher (RTX 4070 Super Recommended)
- Language: Python 3.8+
- Libraries:
  - ultralytics (YOLOv8)
  - ensemble-boxes (WBF)
  - opencv-python
  - tensorrt (Optional)

## How to Run

### 1. Install dependencies

```bash
pip install ultralytics ensemble-boxes opencv-python
```

### 2. Run the main system

```bash
python detection.py
```

## (Macbook / vscode) setup

```bash
#'sar_env'라는 이름으로 가상환경 생성 (Python 3.10 권장)
conda create -n sar_env python=3.10 -y
conda activate sar_env
pip install "numpy<2.0"
pip install ultralytics ensemble-boxes opencv-python

# 실행하려는 폴더로 이동
cd ~/Desktop/github/WIFI-SAR/object_detection/

# 파이썬 실행
python detection.py
```

## TIP

실시간 처리가 필요한 환경은 보통 laptop을 사용하는데, 사양이 충분하다면 frame skip을 하지 않아도 괜찮지만
테스트한 결과, frame skip을 하여도(skip select = 2) 충분히 실시간으로 detection 성능을 보여줌
local video file Analysis를 하는 경우에는 속도보단 정확도가 중요하므로 frame을 skip하지 않고 사용하는 것을 추천

## samll_model

yolov8s로 훈련하여, yolov8m으로 훈련한 모델보다 가벼움
macbook pro m4 기준 frame skip 없이 사용해도 충분히 실시간을 반영할 수 있는 속도 가능
yolov8m으로 훈련한 모델도 동일 조건 기준 frame skip 시 실시간으로 작동하므로, 최대한 yolov8m을 사용하되
그래픽카드가 없고 사양이 낮아 yolov8m 모델을 사용할때 거의 실행 불가능한 경우 이 모델을 사용

## License

This project uses datasets and models based on:

- SARD Dataset: MIT License
  - Source: [https://www.kaggle.com/datasets/nikolasgegenava/sard-search-and-rescue](https://www.kaggle.com/datasets/nikolasgegenava/sard-search-and-rescue)
- VisDrone Dataset: Academic Use Only
- Ultralytics YOLO: AGPL-3.0
