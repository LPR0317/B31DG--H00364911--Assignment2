#include <Arduino.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

// defining I/O pins
#define t1out 23  // T1 digital out pin
#define t2in 18   // T2 digital in pin
#define t3in 19   // T3 digital in pin
#define t4in 14   // T4 analogue in pin
#define t4out 21 // T4 LED pin out
#define t7in 15   // T7 digital in pin
#define t7out 16  // T7 digital out pin

// T5

SemaphoreHandle_t frequencySemaphore;

// freqency calculation on Task2 & 3 square waves
volatile unsigned long freqhigh;

//task7
QueueHandle_t eventQueue;




struct FrequencyData {
  int freqTask2;
  int freqTask3;
} frequencyData;


//max stack : 4096 - stackWaterMark3340  = 756
void Task1(void *pvParameters) {
    UBaseType_t stackWaterMark;
    
    while (1) {
    // 产生脉冲序列
    // Generate pulse sequence
    TickType_t xLastWakeTime = xTaskGetTickCount();
    digitalWrite(t1out, HIGH);
    delayMicroseconds(180);
    digitalWrite(t1out, LOW);
    delayMicroseconds(40);
    digitalWrite(t1out, HIGH);
    delayMicroseconds(530);
    digitalWrite(t1out, LOW);
    delayMicroseconds(250);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4));
    // 每秒检查一次栈使用情况
    // Check the stack usage once per second
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // stackWaterMark = uxTaskGetStackHighWaterMark(NULL); // 检查当前任务的栈使用情况
    // Check the stack usage of the current task
    // Serial.print("Task1 stack high water mark: ");
    // Serial.println(stackWaterMark); // 输出剩余的最小栈空间
    // Output the minimum remaining stack space
}

}

  

//4096 -3481=615
  void Task2(void *pvParameters) {
    const TickType_t xDelay = 20 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 测量高电平脉冲的持续时间 Measure the duration of a high level pulse
        freqhigh = pulseIn(t2in, HIGH, 3000);//voltage
        // 计算频率calculated rate
        if(freqhigh > 0) {
            int tempFreq = 1000000 / (freqhigh * 2);

            // 在修改共享资源前获取信号量
            if (xSemaphoreTake(frequencySemaphore, portMAX_DELAY) == pdTRUE) {
                frequencyData.freqTask2 = tempFreq;
                xSemaphoreGive(frequencySemaphore); // 修改完成，释放信号量Release semaphore
                // Serial.print("Measured Frequency from Task2: ");
                // Serial.println(frequencyData.freqTask2);

            }
        }

        vTaskDelayUntil(&xLastWakeTime, xDelay);

        // 检查当前任务的栈使用情况Stack usage
        UBaseType_t stackWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // Serial.print("Task2 stack high water mark: ");
        // Serial.println(stackWaterMark * sizeof(StackType_t)); // 输出剩余的最小栈空间
    }
}




//4096 - 3180 =916
void Task3(void *pvParameters) {
    const TickType_t xDelay = 8 / portTICK_PERIOD_MS; // 每8ms执行一次
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 测量高电平脉冲的持续时间Measure the duration of a high level pulse
        unsigned long freqHigh = pulseIn(t3in, HIGH, 3000); // 用3000微秒作为超时
        // 若成功测量到高电平脉冲，则计算频率
        if(freqHigh > 0) {
            int tempFreq = 1000000 / (freqHigh * 2); // 使用临时变量计算频率

            // 在修改共享资源前获取信号量
            if (xSemaphoreTake(frequencySemaphore, portMAX_DELAY) == pdTRUE) {
                frequencyData.freqTask3 = tempFreq; // 更新全局结构体中的频率值
                xSemaphoreGive(frequencySemaphore); // 完成修改，释放信号量
                // Serial.print("Measured Frequency from Task2: ");
                // Serial.println(frequencyData.freqTask2);
            }
        }

        // 使用vTaskDelayUntil确保任务周期精确为8ms
        vTaskDelayUntil(&xLastWakeTime, xDelay);

        // 检查当前任务的栈使用情况
        UBaseType_t stackWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // Serial.print("Task3 stack high water mark: ");
        // Serial.println(stackWaterMark * sizeof(StackType_t)); // 输出剩余的最小栈空间（以字节为单位）
    }
}





// stack= 4096 - 3136 = 960
void Task4(void *pvParameters) {
    // 设置为存储最近10次模拟输入读数的数组
    int readings[10] = {0};
    int readIndex = 0;
    long total = 0;
    int average = 0;
    int readValue = 0;

    while (1) {
        // 读取模拟值
        readValue = analogRead(t4in);
        // 更新总和
        total = total - readings[readIndex] + readValue;
        // 将读取的值存储在当前索引处
        readings[readIndex] = readValue;
        // 更新索引：如果达到了数组的末尾，就回到起始处
        readIndex = (readIndex + 1) % 10;
        
        // 计算平均值
        average = total / 10;

        // 如果平均值大于最大范围的一半，点亮LED//
        if (average > (4096 / 2 - 1)) {
            digitalWrite(t4out, HIGH);
        } else {
            digitalWrite(t4out, LOW);
        }
        
        // 等待20ms
        vTaskDelay(pdMS_TO_TICKS(20));

        // 检查当前任务的栈使用情况
        UBaseType_t stackWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // Serial.print("Task4 stack high water mark: ");
        // Serial.println(stackWaterMark * sizeof(StackType_t)); // 输出剩余的最小栈空间（以字节为单位）
    }
}







//4096  3324 = 772
// 增加用于存储历史频率数据的变量，以计算滑动平均
#define HISTORY_SIZE 5
int freqTask2History[HISTORY_SIZE] = {0};
int freqTask3History[HISTORY_SIZE] = {0};
int historyIndex = 0;

void Task5(void *pvParameters) {
    const TickType_t xDelay = 200 / portTICK_PERIOD_MS; // 设置任务执行周期为200ms
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 获取信号量以安全访问共享资源
        if (xSemaphoreTake(frequencySemaphore, portMAX_DELAY) == pdTRUE) {
            // 更新频率历史数据
            freqTask2History[historyIndex] = frequencyData.freqTask2;
            freqTask3History[historyIndex] = frequencyData.freqTask3;
            historyIndex = (historyIndex + 1) % HISTORY_SIZE;

            // 计算两个频率的滑动平均值
            int avgFreqTask2 = 0, avgFreqTask3 = 0;
            for (int i = 0; i < HISTORY_SIZE; i++) {
                avgFreqTask2 += freqTask2History[i];
                avgFreqTask3 += freqTask3History[i];
            }
            avgFreqTask2 /= HISTORY_SIZE;
            avgFreqTask3 /= HISTORY_SIZE;

            // 对滑动平均后的频率进行缩放和限定
            int scaledFreqTask2 = avgFreqTask2 <= 333 ? 0 :
                                  avgFreqTask2 >= 1000 ? 99 :
                                  map(avgFreqTask2, 333, 1000, 0, 99);

            int scaledFreqTask3 = avgFreqTask3 <= 500 ? 0 :
                                  avgFreqTask3 >= 1000 ? 99 :
                                  map(avgFreqTask3, 500, 1000, 0, 99);

            // 释放信号量
            xSemaphoreGive(frequencySemaphore);

            // 通过串行端口输出缩放和限定后的频率值
            Serial.print(scaledFreqTask2);
            Serial.print(",");
            Serial.println(scaledFreqTask3);
        }

        // 等待，直到下次执行
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}






//4096 -3184 =912
void Task7(void *pvParameters) {
    const TickType_t debounceDelay = pdMS_TO_TICKS(50); // 设定50ms为去抖动延时Set 50ms as the dejitter delay
    int buttonState = LOW; // 当前按钮状态，初始化为LOW
    int lastButtonState = LOW; // 上一次读取的按钮状态
    TickType_t lastDebounceTime = 0; // 上次状态改变的时间

    while (1) {
        int reading = digitalRead(t7in); // 读取当前按钮状态

        // 如果按钮状态发生变化
        if (reading != lastButtonState) {
            lastDebounceTime = xTaskGetTickCount(); // 重置去抖动计时器Reset the dejitter timer
        }

        // 如果去抖动时间已过并且按钮状态与上次检测到的状态不同
        if ((xTaskGetTickCount() - lastDebounceTime) > debounceDelay && reading != buttonState) {
            buttonState = reading; // 更新按钮状态

            // 如果当前按钮状态为按下
            if (buttonState == HIGH) {
                int event = 1; // 定义一个事件标志，用1表示按钮被按下
                xQueueSend(eventQueue, &event, 0); // 将事件发送到队列
            }
        }

        lastButtonState = reading; // 更新上一次的按钮状态

        vTaskDelay(pdMS_TO_TICKS(10)); // 稍微延迟以减少CPU占用
        
        // 检查当前任务的栈使用情况
        UBaseType_t stackWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // Serial.print("Task7 stack high water mark: ");
        // Serial.println(stackWaterMark * sizeof(StackType_t)); // 输出剩余的最小栈空间（以字节为单位）
    }

    // 如果任务退出循环，删除任务
    vTaskDelete(NULL);
}

//4096 - 3368 = 728
void ControlLEDTask(void *pvParameters) {
    int event;

    while (1) {
        if (xQueueReceive(eventQueue, &event, portMAX_DELAY) == pdTRUE) {
            // 检查接收到的事件
            if (event == 1) { // 如果是按钮被按下的事件
                digitalWrite(t7out, !digitalRead(t7out)); // 切换LED状态
            }
        }

        // 检查当前任务的栈使用情况
        UBaseType_t stackWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // Serial.print("ControlLEDTask stack high water mark: ");
        // Serial.println(stackWaterMark * sizeof(StackType_t)); // 输出剩余的最小栈空间（以字节为单位）
        
        // 为了避免频繁的串口输出影响性能，可以在实际应用中减少输出频率或去掉输出
    }
}


void CPU_work(int time) {
    // 假设每次循环迭代大约需要1微秒，这取决于CPU和编译器优化
    int iterations = time * 1000; // 例如，time为2，则迭代2000次
    for (int i = 0; i < iterations; i++) {
        // 一个空操作 (no-operation) 的汇编指令，用于消耗时间
        asm volatile("nop");
    }
}
//4096 - 3192 = 904
void CPUWorkTask(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 设置为20ms的周期 Set to a 20ms period
    TickType_t xLastWakeTime = xTaskGetTickCount(); // 获取当前的tick计数 Get the current tick count

    while (1) {
        CPU_work(2); // 模拟CPU工作2ms Simulate CPU work for 2ms

        // 打印任务堆栈剩余空间
        UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // Serial.print("CPUWorkTask Stack Space Remaining: ");
        // Serial.println(uxHighWaterMark * sizeof(StackType_t)); // 如果您的平台是32位的，一个StackType_t通常是4字节 If your platform is 32-bit, a StackType_t is typically 4 bytes

        // 延迟直到下一个周期 Delay until the next period
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}



void setup()
{
    // // set baud rate 9600
    Serial.begin(9600);
    frequencySemaphore = xSemaphoreCreateMutex();
    if (frequencySemaphore == NULL) {
        // 处理信号量创建失败的情况2.3.5
        Serial.println("Failed to create semaphore");
    }

    // // initialise output pins for Task1 & 4
    pinMode(t1out, OUTPUT);
    pinMode(t4out, OUTPUT);
    pinMode(t7out, OUTPUT);

    // // initialise input pins for Task2, 3 & 4
    pinMode(t2in, INPUT);
    pinMode(t3in, INPUT);
    pinMode(t4in, INPUT);
    pinMode(t7in, INPUT_PULLUP); // 将按钮引脚设置为输入上拉

    // 初始化事件队列Initializes the event queue
    eventQueue = xQueueCreate(10, sizeof(int));
    if (eventQueue == NULL) {
        Serial.println("Error creating the queue");
        // 处理错误情况
    }

    //  create tasks
  xTaskCreate(Task1, "Task1", 4096, NULL, 3, NULL); // 高优先级
  xTaskCreate(Task2, "Task2", 4096, NULL, 2, NULL); // 中优先级
  xTaskCreate(Task3, "Task3", 4096, NULL, 2, NULL); // 中优先级
  xTaskCreate(Task4, "Task4", 4096, NULL, 2, NULL); // 中优先级
  xTaskCreate(Task5, "Task5", 1000, NULL, 1, NULL); // 低优先级
  xTaskCreate(Task7, "Task7", 4096, NULL, 1, NULL); // 低优先级
  xTaskCreate(CPUWorkTask, "CPUWorkTask", 4096, NULL, 1, NULL);
  xTaskCreate(ControlLEDTask, "ControlLEDTask", 4096, NULL, 1, NULL);

  
}

void loop()
{
   
}
