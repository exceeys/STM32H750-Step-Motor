#include "main.h"
#include "tim.h"
#include "StepperMotor.h"

#include <math.h>

#define ABS(x)		((x < 0) ? -x : x)

int32_t Distance;								//number of step
float 	Speed = 10.0f;					//target velocity (unit per second),
int32_t Acceleration = 100;			//acceleration    (unit per second per second),

int32_t	PositionCount = 0;
int     Direction;
int			Busy;

int32_t A;
float   R;											//constant multiplier
float   DelayPeriod;

int32_t DistanceNow;						//number of steps
int32_t DistanceAcceleration;		//number of steps

// 이동이 끝났을 때 불려지는 콜백함수이다.  재정의해서 사용하면 된다.
__weak void MoveEndCallback(void)
{

}

// 모터 드라이버의 Enable을 활성화 시킨다.
void EnableMotor(void)
{
	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);
}

// 모터 드라이버의 Enable을 비활성화 시킨다.
void DisableMotor(void)
{
	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);
}

// 모터를 갑자기 멈춘다.
void StopEMS(void)
{
	DistanceNow = Distance;
}

// 기본 값을 초기값으로 채운다.
void InitStepperMotor(float speed, int32_t acc)
{
	Speed = speed;
	Acceleration = acc;

	PositionCount = 0;
	Direction = DIRECTION_FORWARD;

  __HAL_TIM_CLEAR_IT(&IntervalTimer, TIM_IT_UPDATE);	// 혹시 남아 있을 지 모르는 인터럽트 플래그를 지운다.
  __HAL_TIM_CLEAR_IT(&PulsTimer, TIM_IT_UPDATE);
}

// 속도 값을 설정한다.
// 예를 들어 유닛이 mm이고 10으로 설정하였다면 초당 10mm를 움직이는 속도이다.
void SetSpeed(float speed)
{
	Speed = speed;
}

// 설정된 현재 속도 값을 읽는다.
float GetSpeed(void)
{
	return Speed;
}

// 가속도 값을 설정한다.
void SetAcceleration(int32_t acc)
{
	Acceleration = acc;
}

// 설정된 현재 가속도 값을 읽는다.
int32_t GetAcceleration(void)
{
	return Acceleration;
}

// 현재 위치를 얻는다.
float GetPosition(void)
{
	return (float)PositionCount / STEPS_PER_UNIT;
}

// 현재 위치를 재설정한다.
void SetPosition(float position)
{
	PositionCount = position * STEPS_PER_UNIT;
}

// 현재 상태를 얻는다.
int GetState(void)
{
	return Busy;
}

// 타이머 콜백 함수
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == IntervalTimer.Instance)	// 펄스 주기 타이머
	{
		if(DistanceNow < Distance) // 주행 중
		{
				DistanceNow++;
				if (DistanceNow < DistanceAcceleration)	// 가속 구간
				{
					DelayPeriod = DelayPeriod * (1.0f - R * DelayPeriod * DelayPeriod);
				}
				else if (DistanceNow >= Distance - DistanceAcceleration) // 감속 구간
				{
					DelayPeriod = DelayPeriod * (1.0f + R * DelayPeriod * DelayPeriod);
				}
				else		// 정속 구간
				{
				}

				if(DelayPeriod > 0xFFFF ) DelayPeriod = 0xFFFF;			// 16bit timer 범위를 벗어나지 않게
				IntervalTimer.Instance->ARR = DelayPeriod;

				if(Direction == DIRECTION_FORWARD) PositionCount++; else PositionCount--; // 현재 위치 업데이트

				HAL_TIM_Base_Start_IT(&PulsTimer);
				HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_SET);	// 펄스를 시작한다.
		}
		else // 주행 완료
		{
			HAL_TIM_Base_Stop(&IntervalTimer);
			Busy = STATE_READY;

			MoveEndCallback();
		}
	}
	else if(htim->Instance == PulsTimer.Instance)  // 펄스 폭 타이머
	{
		HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);   // 펄스를 끝낸다.
		HAL_TIM_Base_Stop(&PulsTimer);
	}
}

// 상대적 거리만큼 움직이는 함수
void MoveDistance(float distance)
{
	int32_t acc;

	if(ABS(distance) < 1.0f / STEPS_PER_UNIT) return;
	if(Speed == 0.0f) return;
	if(Acceleration == 0) return;

	Busy = STATE_BUSY;

	// 방향을 결정한다.
	if(distance >= 0.0f)
	{
			Direction = DIRECTION_FORWARD;
			HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
	}
	else
	{
		distance *= -1.0f;
		Direction = DIRECTION_REVERSE;
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
	}

	// 목표 속도에 이르는 가속 구간을 계산한다.
  A = Acceleration * STEPS_PER_UNIT;
  Distance = distance * STEPS_PER_UNIT;
  DistanceAcceleration = (Speed * Speed * STEPS_PER_UNIT * STEPS_PER_UNIT) / (2 * A);


#if 1
  // 가속 구간이 최소한 10 스텝은 되게. 그래야 저속에서 최종 속도가 어느 정도 맞음.
  if(DistanceAcceleration < 10)
  {
  	acc = (Speed * Speed * STEPS_PER_UNIT) / 20;
    A = acc * STEPS_PER_UNIT;
    DistanceAcceleration = (Speed * Speed * STEPS_PER_UNIT * STEPS_PER_UNIT) / (2 * A);
  }
#endif

  // 전체 거리가 가속 및 감속 구간의 합보다 짧으면 감가속구간을 거리의 반으로 설정
  if (DistanceAcceleration >= (Distance / 2))
  {
  	DistanceAcceleration = (Distance / 2);
  }
  R = (double)A / ((int64_t)FrequencyTime * FrequencyTime);
  DelayPeriod = FrequencyTime / sqrtf(2 * A);
  if(DelayPeriod > 0xFFFF) DelayPeriod = 0xFFFF; // 16bit 타이머일 경우 범위를 넘어가지 않게

  PulsTimer.Instance->CNT = 0;
  IntervalTimer.Instance->CNT = 0;
  IntervalTimer.Instance->ARR = DelayPeriod;
  HAL_TIM_Base_Start_IT(&IntervalTimer);

#if 0
	DistanceNow = 0;
#else
	// 처음 부터 펄스를 하나 발생 시키고 시작
	DistanceNow = 1;
	if(Direction == DIRECTION_FORWARD) PositionCount++; else PositionCount--;
	HAL_TIM_Base_Start_IT(&PulsTimer);
	HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_SET);
#endif
}

// 절대 위치로 움직이는 함수
void MovePosition(float position)
{
	MoveDistance(position - GetPosition());
}

