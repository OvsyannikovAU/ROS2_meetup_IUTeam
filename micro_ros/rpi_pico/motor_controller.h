
#include <stdint.h>
#include <stdbool.h>
#include "hardware/gpio.h"


/* @brief Параметры PID-регулятора.
 */
typedef struct {
    float kp;   ///< Пропорциональный коэффициент
    float ki;   ///< Интегральный коэффициент
    float kd;   ///< Дифференциальный коэффициент
    float kff;  ///< Feed-forward коэффициент (при необходимости)
    float integral;    ///< Накопленная интегральная часть
    float prev_error;  ///< Ошибка на предыдущем шаге
} PIDController;

/* @brief Упрощённая структура для передачи PID-параметров одним блоком.
 * Используется, если нужно обновлять PID «на лету» через сообщения.
 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float kff;
} PIDParams;

/* @brief Основная структура, описывающая мотор, пины, счёт энкодера, PID и текущую скорость.
 */
typedef struct {
    uint32_t pwm_pin;      ///< GPIO пин для ШИМ-сигнала
    uint32_t dir1_pin;     ///< GPIO пин для направления (DIR1)
    uint32_t dir2_pin;     ///< GPIO пин для направления (DIR2)
    uint32_t enc_a_pin;    ///< GPIO пин канала A энкодера
    uint32_t enc_b_pin;    ///< GPIO пин канала B энкодера

    PIDController pid;     ///< Текущие коэффициенты PID
    float target_rpm;      ///< Желаемая скорость вращения (об/мин)
    float current_rpm;     ///< Текущая оценка скорости (об/мин)

    volatile int32_t encoder_count; ///< Текущий счётчик импульсов энкодера
    int32_t prev_encoder_count;     ///< Сохранённое значение счётчика на предыдущем шаге
    uint64_t prev_time_us;          ///< Время предыдущего обновления, мкс
} Motor;

/* @brief Тип функции-колбэка, вызываемой при изменении (фронте) сигналов энкодера.
 * @param motor     Указатель на структуру Motor, к которой привязан энкодер
 * @param direction Направление вращения (true/false)
 */
typedef void (*encoder_callback_t)(Motor* motor, bool direction);

/* @brief Зарегистрировать пользовательский колбэк для обработки фронтов энкодера.
 * По умолчанию регистрируется внутренний колбэк, который просто считает +1/-1.
 *
 * @param motor     Указатель на структуру Motor
 * @param callback  Указатель на функцию-колбэк
 */
void Motor_RegisterEncoderCallback(Motor* motor, encoder_callback_t callback);

/* @brief Инициализация мотора, привязка пинов, настройка ШИМ, настройка GPIO, настройка энкодера.
 *
 * @param motor      Указатель на структуру Motor
 * @param pwm_pin    GPIO пин для ШИМ
 * @param dir1_pin   GPIO пин направления #1
 * @param dir2_pin   GPIO пин направления #2
 * @param enc_a_pin  GPIO пин энкодера (канал A)
 * @param enc_b_pin  GPIO пин энкодера (канал B)
 */
void Motor_Init(Motor* motor,
                uint32_t pwm_pin,
                uint32_t dir1_pin,
                uint32_t dir2_pin,
                uint32_t enc_a_pin,
                uint32_t enc_b_pin);

/* @brief Установить PID-параметры (kp, ki, kd, kff) из структуры PIDParams.
 *
 * @param motor   Указатель на структуру Motor
 * @param params  Указатель на структуру PIDParams
 */
void Motor_SetPID(Motor* motor, const PIDParams* params);

/* @brief Установить целевую скорость вращения (об/мин).
 *
 * @param motor  Указатель на структуру Motor
 * @param rpm    Желаемая скорость, об/мин
 */
void Motor_SetTargetRPM(Motor* motor, float rpm);

/* @brief Получить текущую оценку скорости вращения (об/мин).
 *
 * @param motor  Указатель на структуру Motor
 * @return Текущая скорость, об/мин
 */
float Motor_GetCurrentRPM(const Motor* motor);

float Motor_GetCurrentENC(const Motor* motor);


/* @brief Получить целевую скорость вращения (об/мин).
 *
 * @param motor  Указатель на структуру Motor
 * @return Целевая скорость, об/мин
 */
float Motor_GetTargetRPM(const Motor* motor);

/**
 * @brief Основная функция обновления PID-регулятора и ШИМ, вызвается периодически в основном цикле.
 *
 * @param motor      Указатель на структуру Motor
 * @param dt         Время (в секундах) с момента предыдущего вызова
 * @param correction Дополнительная корректирующая величина, если нужно
 */
void Motor_Update(Motor* motor, float dt, float correction);
