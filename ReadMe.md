LF:2
PWM  PA8
IN1  PC0
IN2  PC1
ENC  PA5 PA1

RF:1
PWM  PE5
IN1  PC2
IN2  PC3
ENC  PA6 PA7

LR:4
PWM  PF6
IN1  PC4
IN2  PC5
ENC  PD12 PD13

RR:3
PWM  PF8
IN1  PD2
IN2  PG11
ENC  PC6 PC7


RF  pid kp=8.5 ki=4.0 kd=0.0 ff=1.1 tgt=175.0 ilim=50 sep=50.0 flt=0.30

PD6 PD5


M0 
    Kp = 5.0
    Kff = 1.20
    Ki = 0
    Kd = 0

#	UART	TX	RX	AF	×ÜÏß	Ê±ÖÓ
1	USART2	PA2	PA3	AF7	APB1	42MHz                   ESP32 wroom
2	USART3	PB10	PB11	AF7	APB1	42MHz                   IMU
3	UART4	PC10	PC11	AF8	APB1	42MHz                   LIDAR