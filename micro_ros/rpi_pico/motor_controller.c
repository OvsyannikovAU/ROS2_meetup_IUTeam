#include "motor_controller.h"

#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include <stdio.h>

/* 
 * Константа «импульсов на оборот» для энкодера.
 * Если у вас энкодер 11 CPR и передаточное отношение 30:1, 4 события RISE/FALLS.
 */
#define ENCODER_PULSES_PER_REV 1320.0f

/* @brief Максимально допустимое число моторов, которые можно обслуживать в ISR.
 * При желании увеличить, изменить здесь.
 */
#define MAX_MOTORS 2

/* @brief Статические массивы для хранения зарегистрированных моторов и их колбэков.
 */
static Motor* s_registered_motors[MAX_MOTORS] = {NULL};
static encoder_callback_t s_encoder_callbacks[MAX_MOTORS] = {NULL};

/* @brief Колбэк по умолчанию: считает счётчик +1/-1, основываясь на направлении.
 */
static void default_encoder_callback(Motor* motor, bool direction)
{
    // direction=true => +1, иначе -1
    motor->encoder_count += direction ? 1 : -1;
}

/* @brief Обработка прерывания от энкодера. 
 * Вызывается при любом фронте (RISING или FALLING) на A или B.
 */
static void encoder_isr(uint32_t gpio, uint32_t events)
{
    // Перебираем зарег. моторы
    for (int i = 0; i < MAX_MOTORS; i++) {
        Motor* motor = s_registered_motors[i];
        if (motor == NULL) {
            continue;
        }

        // Проверяем, относится ли прерывание к энкодеру этого мотора
        if (gpio == motor->enc_a_pin || gpio == motor->enc_b_pin) {
            // Считываем текущее состояние обеих линий
            bool a_state = gpio_get(motor->enc_a_pin);
            bool b_state = gpio_get(motor->enc_b_pin);

            // Определяем направление
            // Вариант: если фронт на A, то direction = (a_state == b_state);
            // Если фронт на B, то direction = (a_state != b_state);
            // Но проще просто оставить исходную проверку:
            bool direction;
            if (gpio == motor->enc_a_pin) {
                direction = (a_state == b_state);
            } else {
                direction = (a_state != b_state);
            }

            // Вызываем колбэк, если он есть
            if (s_encoder_callbacks[i]) {
                s_encoder_callbacks[i](motor, direction);
            }
            break; 
        }
    }
}

void Motor_RegisterEncoderCallback(Motor* motor, encoder_callback_t callback)
{
    // Ищем свободный слот
    for (int i = 0; i < MAX_MOTORS; i++) {
        if (s_registered_motors[i] == NULL) {
            s_registered_motors[i] = motor;
            s_encoder_callbacks[i] = callback;

            // Настраиваем прерывания на обеих линиях (все фронты)
            gpio_set_irq_enabled_with_callback(motor->enc_a_pin,
                GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                true,
                &encoder_isr);

            gpio_set_irq_enabled(motor->enc_b_pin,
                GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                true);

            return;
        }
    }
    // Если дошли сюда, значит массив заполнен
    // Можно вывести предупреждение (printf или лог).
    printf("Warning: No free slots for registering motor callback!\n");
}

void Motor_Init(Motor* motor,
                uint32_t pwm_pin,
                uint32_t dir1_pin,
                uint32_t dir2_pin,
                uint32_t enc_a_pin,
                uint32_t enc_b_pin)
{
    // Сохраняем пины
    motor->pwm_pin  = pwm_pin;
    motor->dir1_pin = dir1_pin;
    motor->dir2_pin = dir2_pin;
    motor->enc_a_pin = enc_a_pin;
    motor->enc_b_pin = enc_b_pin;

    // Инициализация GPIO под ШИМ и направление
    gpio_init(pwm_pin);
    gpio_set_dir(pwm_pin, GPIO_OUT);

    gpio_init(dir1_pin);
    gpio_set_dir(dir1_pin, GPIO_OUT);

    gpio_init(dir2_pin);
    gpio_set_dir(dir2_pin, GPIO_OUT);

    // Включаем функцию PWM на нужном пине
    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);

    // Получаем "slice"
    // для этого пина и конфигурируем
    uint32_t slice_num = pwm_gpio_to_slice_num(pwm_pin);
    pwm_set_wrap(slice_num, 255);       // 8-битный PWM (0..255)
    pwm_set_enabled(slice_num, true);   // Включаем генерацию PWM

    // Инициализация пинов энкодера
    gpio_init(enc_a_pin);
    gpio_set_dir(enc_a_pin, GPIO_IN);
    gpio_pull_up(enc_a_pin);
    gpio_set_input_hysteresis_enabled(enc_a_pin, true);

    gpio_init(enc_b_pin);
    gpio_set_dir(enc_b_pin, GPIO_IN);
    gpio_pull_up(enc_b_pin);
    gpio_set_input_hysteresis_enabled(enc_b_pin, true);

    // Сбрасываем возможные «подвисшие» флаги прерывания
    gpio_acknowledge_irq(enc_a_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    gpio_acknowledge_irq(enc_b_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);

    // Регистрируем колбэк по умолчанию
    Motor_RegisterEncoderCallback(motor, &default_encoder_callback);

    // Инициализация PID и прочих переменных
    motor->pid.kp         = 0.0f;
    motor->pid.ki         = 0.0f;
    motor->pid.kd         = 0.0f;
    motor->pid.kff        = 0.0f;
    motor->pid.integral   = 0.0f;
    motor->pid.prev_error = 0.0f;

    motor->target_rpm     = 0.0f;
    motor->current_rpm    = 0.0f;
    motor->encoder_count  = 0;
    motor->prev_encoder_count = 0;
    motor->prev_time_us   = time_us_64();
}

void Motor_SetPID(Motor* motor, const PIDParams* params)
{
    if (!motor || !params) {
        return;
    }
    motor->pid.kp   = params->kp;
    motor->pid.ki   = params->ki;
    motor->pid.kd   = params->kd;
    motor->pid.kff  = params->kff;

    // Сбрасываем интеграл и предыдущую ошибку
    motor->pid.integral   = 0.0f;
    motor->pid.prev_error = 0.0f;
}

void Motor_SetTargetRPM(Motor* motor, float rpm)
{
    if (!motor) {
        return;
    }
    motor->target_rpm = rpm;
}

float Motor_GetCurrentRPM(const Motor* motor)
{
    if (!motor) {
        return 0.0f;
    }
    return motor->current_rpm;
}


float Motor_GetCurrentENC(const Motor* motor)
{
    if (!motor) {
        return 0.0f;
    }
    return motor->encoder_count;
}

float Motor_GetTargetRPM(const Motor* motor)
{
    if (!motor) {
        return 0.0f;
    }
    return motor->target_rpm;
}

void Motor_Update(Motor* motor, float dt, float correction)
{
    if (!motor || dt < 1e-6f) { 
        return; 
    }

    // Атомарно читаем encoder_count, защищаемся от одновременной записи в ISR
    int32_t current_count;
    uint32_t irq_status = save_and_disable_interrupts();
    current_count = motor->encoder_count;
    restore_interrupts(irq_status);

    // Считаем разницу
    int32_t delta_counts = current_count - motor->prev_encoder_count;
    motor->prev_encoder_count = current_count;

    // Оцениваем текущую скорость (RPM)
    // PULSE_PER_REV -> сколько импульсов на один оборот.
    // за dt секунд набрано delta_counts импульсов => (delta_counts / pulses_per_rev) оборотов
    // в одну секунду будет (delta_counts / pulses_per_rev) / dt оборотов => умножаем на 60 для RPM
    float new_rpm = (delta_counts / ENCODER_PULSES_PER_REV) * (60.0f / dt);
    motor->current_rpm = new_rpm;

    // Выполняем PID
    float error = (motor->target_rpm - motor->current_rpm) + correction;
    motor->pid.integral += error * dt;
    float derivative = (error - motor->pid.prev_error) / dt;

    float output = (motor->pid.kp * error)
                 + (motor->pid.ki * motor->pid.integral)
                 + (motor->pid.kd * derivative)
                 + (motor->pid.kff * motor->target_rpm); // опционально можно выключать

    motor->pid.prev_error = error;

    // Ограничиваем выход PWM в диапазоне 0..255 (но с учётом направления, значит -255..255)
    if (output > 255.0f) {
        output = 255.0f;
    } else if (output < -255.0f) {
        output = -255.0f;
    }

    // Настраиваем направления
    bool forward = (output >= 0.0f);
    float pwm_value = forward ? output : -output; // Модуль

    gpio_put(motor->dir1_pin, forward ? 1 : 0);
    gpio_put(motor->dir2_pin, forward ? 0 : 1);

    // Устанавливаем уровень PWM (8-бит, от 0 до 255)
    uint32_t slice_num = pwm_gpio_to_slice_num(motor->pwm_pin);
    pwm_set_gpio_level(motor->pwm_pin, (uint16_t)pwm_value);
}