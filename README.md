
# 빌드 방법

### 윈도우에서서 우분투 위치 찾기
```
\\wsl$\Ubuntu-20.04\
```

### 원클릭 자동화 스크립트

1. `run.sh` 파일 작성 및 저장
```sh
#!/bin/bash

echo "[1] 빌드 시작"
cd ~/xycar_ws || exit
catkin_make || { echo "catkin_make 실패"; exit 1; }
source devel/setup.bash

echo "[2] roslaunch로 TCP 통신 노드 실행 (백그라운드)"
source devel/setup.bash
roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=0.0.0.0 tcp_port:=10000 &
ROSLAUNCH_PID=$!

sleep 2  # roslaunch 초기화 대기

echo "[3] track_drive.py 실행"
python3 ~/xycar_ws/src/kookmin/driver/track_drive.py

# (선택) 종료 시 roslaunch도 정리
kill $ROSLAUNCH_PID
```


2. 실행권한 부여
```
$ chmod +x run.sh
```

3. 실행
```
$ ./run.sh
```





# 아래내용들은 전부 위에 포함되어있음




### 빌드
```sh
cd ~/xycar_ws
catkin_make
source devel/setup.bash
```

### TCP 통신 수행
```sh
roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=0.0.0.0 tcp_port:=10000
```
### 실행
```sh
python3 ~/xycar_ws/src/kookmin/driver/track_drive.py
```