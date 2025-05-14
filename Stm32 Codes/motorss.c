{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); //left motor1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //left motor2
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); //right motor1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //right motor2

  uint32_t speed = htim2.Init.Period*0.2;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //double motor standby
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,0);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
	  HAL_Delay(5000);
	  //left motor back
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,speed);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
	  //right motor  back
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,speed);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
	  HAL_Delay(2000);
	  //double motor brake
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,speed);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,speed);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,speed);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,speed);
	  HAL_Delay(2000);
	  //left motor forward
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,0);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,speed);
	  //right motor forward
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,speed);
	  HAL_Delay(2000);
	  //double motor brake
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,0);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);

  }
  /* USER CODE END 3 */
}