/* Structural fixture: the relevant shape CubeMX regenerates from Tasks01. */
osThreadId_t appMainTaskHandle;
static StaticTask_t appMainTaskControlBlock;
static uint32_t appMainTaskBuffer[512];
const osThreadAttr_t appMainTask_attributes = {
  .name = "appMainTask",
  .cb_mem = &appMainTaskControlBlock,
  .cb_size = sizeof(appMainTaskControlBlock),
  .stack_mem = &appMainTaskBuffer[0],
  .stack_size = sizeof(appMainTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

int regenerated_main_fixture(void)
{
  osKernelInitialize();
  appMainTaskHandle = osThreadNew(app_main_task, NULL, &appMainTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if (!app_bootstrap_create())
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  osKernelStart();
}
