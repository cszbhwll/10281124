# 电脑端模拟器：在 sim 目录下执行 make，然后 ./sim
# Windows 可用 MSYS2/MinGW 的 gcc，或 WSL。
# 模拟器自带"模拟陀螺仪"（world.c），所以这里关掉真实 MPU6050 驱动。
CC      ?= gcc
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra -Wno-unused-parameter -D_DEFAULT_SOURCE -DUSE_MPU6050=0 -I../Core/Inc -I.
APP_SRC  = imu_mpu6050.c sys.c field.c motor_drv.c chassis.c nav.c bus_servo.c arm.c vision.c display.c task_code.c mission.c console.c
SRC      = $(addprefix ../Core/Src/,$(APP_SRC)) world.c port_sim.c sim_main.c
TEST_SRC = $(addprefix ../Core/Src/,task_code.c) test_task.c

all: sim test_task

sim: $(SRC) $(wildcard ../Core/Inc/*.h) sim.h replay_template.h
	$(CC) $(CFLAGS) -o $@ $(SRC) -lm

test_task: $(TEST_SRC)
	$(CC) $(CFLAGS) -o $@ $(TEST_SRC) -lm

clean:
	rm -f sim test_task
