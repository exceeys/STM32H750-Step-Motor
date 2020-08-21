#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H


#define FrequencyTime 		1000000		//count of timer   (ticks per second)
// 타이머 클럭을 1MHz에 맞춘다.  즉, 240MHz의 CPU일 경우 Prescaler의 값을 240-1로 설정한다.

#define STEPS_PER_UNIT 		1280	//100
// 한 유닛(예;1mm)을 움직이는 데, 몇 스텝이 필요한 지 계산하여 정한다.
// 모터의 펄스당 회전 각도, 감속비, 마이크로 스텝 설정 등을 계산하여야 한다.

#define DIRECTION_FORWARD	0
#define DIRECTION_REVERSE	1

#define STATE_READY				0
#define STATE_BUSY				1

#define IntervalTimer	htim5		// 스텝 주기를 결정할 타이머. 타이머 클럭을 1MHz에 맞춘다. 인터럽트 우선순위를 높게해야 틱틱거리지 않는다.
#define PulsTimer			htim6		// 펄스의 폭을 결정할 타이머. 펄스 폭은 5us 정도로 한다.

void InitStepperMotor(float speed, int32_t acc);		// 초기 값을 설정한다.
void StopEMS(void);																	// 동작 중에 모터를 정지한다.
void MoveDistance(float distance);									// 상대 위치로 움직인다.
void MovePosition(float position);									// 절대 위치로 움직인다.
void MoveEndCallback(void);													// 이동이 끝나면 호출되는 콜백 함수
void SetSpeed(float speed);													// 이동 속도를 설정한다. speed는 초당 움직이는 단위 유닛
float GetSpeed(void);																// 현재 설정된 속도를 가져온다.
void SetAcceleration(int32_t acc);									// 가속도 값을 설정한다.
int32_t GetAcceleration(void);											// 현재 설정된 가속도 값을 가져온다.
float GetPosition(void);														// 현재 위치를 읽어온다.
void SetPosition(float position);										// 현재의 위치 값을 재설정한다.
int GetState(void);																	// 모터의 상태(Busy, Ready)를 가져온다.
void EnableMotor(void);															// 모터 드라이버의 Enable을 활성화 시킨다.
void DisableMotor(void);														// 모터 드라이버의 Enable을 비활성화 시킨다.
/*
 // 스텝 펄스 핀과 방향 핀을 정의한다.
 // STM32Cube에서 핀 이름을 "STEP", "DIR"로 설정하였다면 따로 손대지 않아도 된다.
 // 핀은 'GPIO_Output'으로 설정하여야 한다.
#define STEP_Pin 					GPIO_PIN_5
#define STEP_GPIO_Port 		GPIOC
#define DIR_Pin 					GPIO_PIN_4
#define DIR_GPIO_Port 		GPIOC
*/


#endif // __STEPPER_MOTOR_H
