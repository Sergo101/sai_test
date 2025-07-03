/*!

 *
 */

#ifndef __UTILS_H_
#define __UTILS_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>



#ifndef __FILE_NAME__
  #define __FILE_NAME__ "utils.h"
#endif

#ifndef   __STATIC_FORCEINLINE
  #define __STATIC_FORCEINLINE                   __attribute__((always_inline)) static inline
#endif

#ifndef NO_INIT_SECTION
#define NO_INIT_SECTION ".bss.NoInit"
#endif


#ifdef __cplusplus
extern "C" {
#endif /* #ifdef __cplusplus */

/*!
  \ingroup HAL_GEN
  \{
*/
#ifndef MODULE_TEST
/*!
  \brief Макрос сохранения текущего статуса и выключения всех прерываний
  \note primask типа uint32_t
  \param[in] primask переменная хранения текущей маски всех прерываний
*/
#define DIS_IRQ(primask) do { primask = __get_PRIMASK(); __sync_synchronize(); __disable_irq(); __sync_synchronize(); } while (0)

/*!
  \brief Макрос восстановления статуса прерываний
  \note primask типа uint32_t
  \param[in] primask переменная хранения текущей маски всех прерываний
*/
#define EN_IRQ(primask)  do { __sync_synchronize(); __set_PRIMASK(primask); __sync_synchronize(); } while (0)
#endif /* #ifndef MODULE_TEST */

/*!
  \brief Макрос приоритетного состояния выражения - true
  \note использовать при ветвлении при наиболее частом значении выражения - true
        Пример: if (likely(buff!=NULL)) {do....}
  \param[in] stat выражение
*/
#ifndef likely
  #define likely(stat) __builtin_expect((long)(stat), (long)1)
#endif

/*!
  \brief Макрос приоритетного состояния выражения - false
  \note использовать при ветвлении при наиболее частом значении выражения - false
        Пример: if (unlikely(isInit==0)) {exit error}
  \param[in] stat выражение
*/
#ifndef unlikely
  #define unlikely(stat) __builtin_expect((long)(stat), (long)0)
#endif

/// Макрос атрибута слабосвязанной функции
#ifndef __weak
  #define __weak __attribute__((__weak__))
#endif

#ifndef __aligned
  #define	__aligned(x)	__attribute__((__aligned__(x)))
#endif

#define VAR_DEF(scope, var) decltype(scope::var) scope::var;
/*!
  \brief Переход в обработчик assert
  \param[in] module модуль где произошел assert
  \param[in] file файл где произошел assert
  \param[in] line строка в файле где произошел assert
  \param[in] function функция где произошел assert
  \param[in] msg сообщения assert
*/
#define ASSERT_HANDLER(module, file, line, function, msg) assert_func(module, file, line, function, msg)

/*!
  \brief Базовый assert уровень модуля MCU_ASSERT, без диагностического сообщения
*/
#define ASSERT()                                  ASSERT_HANDLER(0U, __FILE_NAME__, __LINE__, __FUNCTION__, NULL)

/*!
  \brief Проверка валидности условия
  \note При невыполнении заданного утверждения (условия) вызывается функция обработчик assert_func
  \param[in] __e заданное утверждение (условие), 0 - ошибка присутствует, 1 - ошибка отсутствует
*/
#define assert_mcu_release(__e)                   do {if (unlikely(!(__e))) ASSERT(); } while(0)

#if (defined DEBUG) || (defined DOXYGEN_SHOULD_SKIP_THIS)
  /*!
    \brief Форматный вывод текста
    \warning Используется только в режиме отладки (при объявленном макросе DEBUG).
  */
  #define PRINTF(format, args...)                 printf(format, args)

  /*!
    \brief Вывод текста
    \warning Используется только в режиме отладки (при объявленном макросе DEBUG).
  */
  #define PRINT(text)                             printf(text)

  /*!
    \brief Проверка валидности условия
    \note При невыполнении заданного утверждения (условия) вызывается функция обработчик assert_func
    \warning Используется только в режиме отладки (при объявленном макросе DEBUG).
  */
  #define assert_mcu_debug(__e)                   assert_mcu_release(__e)
#else
  #define PRINTF(format, args...)                 ((void)0)
  #define PRINT(text)                             ((void)0)
  #define assert_mcu_debug(__e)                   ((void)0)
#endif

/*!
  \brief Макрос для быстрой проверки условий и возврату
  \param[in] cond условие
  \param[in] retval возвращаемое значение при выполнении условия
*/
#define returnifval(cond, retval)                 do { if (cond) return (retval); } while (0)
/*!
  \brief Макрос для быстрой проверки условий и возврату
  \param[in] cond условие
*/
#define returnif(cond)                            do { if (cond) return; } while (0)

/// Макрос вычисления остатка от деления при однократном переполнении
#define MOD(DIVIDEND, DIVIDER)                    do { if (unlikely(DIVIDEND) >= (DIVIDER)) \
                                                    (DIVIDEND) -= (DIVIDER);} while(0)
#define MAX(X, Y)                                 (((X) > (Y)) ? (X) : (Y))                                             ///< Макрос определения максимума
#define MIN(X, Y)                                 (((X) < (Y)) ? (X) : (Y))                                             ///< Макрос определения минимума
#define DIF(X, Y)                                 (((X) > (Y)) ? (X) - (Y) : (Y) - (X))                                 ///< Макрос разницы двух значений

#define GET_BIT_MCU(reg,bit)                      (((reg) >> (bit)) & 1U )                                              ///<Получение значения бита
#define CLEAR_BIT_MCU(reg,bit)                    ((reg) &= ~(1U << (bit)))                                             ///< Очистка значения бита
#define SET_BIT_MCU(reg,bit)                      ((reg) |= (1U << (bit)))                                              ///< Установка значения бита
#define INVERT_BIT_MCU(reg,bit)                   ((reg) ^= (1U << (bit)))                                              ///< Инвертирование значения бита
#define CHECK_BIT_MCU(reg,bit)                    ((reg) & (1U << (bit)))                                               ///< Проверка значения бита

#define CLEAR_MASK_MCU(reg,mask)                  ((reg) &= ~(mask))                                                    ///< Очистка значения по маске
#define SET_MASK_MCU(reg,mask)                    ((reg) |= (mask))                                                     ///< Установка значения по маске
#define INVERT_MASK_MCU(reg,mask)                 ((reg) ^= (mask))                                                     ///< Инвертирование значения по маске
#define CHECK_MASK_MCU(reg,mask)                  ((reg) & (mask))                                                      ///< Проверка значения по маске
#define CHANGE_MASK_MCU(reg,mask,value)           ((reg) = (((reg) & ~(mask)) | ((mask) & (value))))                    ///< Установка значения по маске
#define CHANGE_BIT_MCU(reg, bit, bit_value)       ((reg) = ((reg) & ~(1U << (bit))) | (((bit_value) & 1U) << (bit)))    ///< Установка значения бита
#define COPY_BIT_MCU(dst, src, bit)               ((dst) = ((dst) & ~(1U << (bit))) | ((src) & (1U << (bit))))          ///< Копирование бита bit из src в dst

/// Установка сетевого порядка байт (16 бит)
#define CT_HTONS(x)                               ((((uint32_t)(x) >> 8U) & (uint16_t)0x00FFU) |      \
                                                   (((uint32_t)(x) << 8U) & (uint16_t)0xFF00U))

/// Установка сетевого порядка байт (32 бита)
#define CT_HTONL(x)                               ((((uint32_t)(x) >> 24U) & (uint32_t)0x000000FFU) | \
                                                   (((uint32_t)(x) >> 8U) & (uint32_t)0x0000FF00U) |  \
                                                   (((uint32_t)(x) << 8U) & (uint32_t)0x00FF0000U) |  \
                                                   (((uint32_t)(x) << 24U) & (uint32_t)0xFF000000U))
/// Установка сетевого порядка байт для констант
#define CT_HTONS_CONST(x)                         CT_HTONS(x)
#define CT_HTONL_CONST(x)                         CT_HTONL(x)

#define U16_TO_STR(val)                           ((((uint32_t)u4_to_hex_char[((uint8_t)(val) & 0x0F)]) << 24UL) +          \
                                                   (((uint32_t)u4_to_hex_char[(((uint8_t)(val) >> 4UL) & 0x0F)]) << 16UL) + \
                                                   (((uint32_t)u4_to_hex_char[(((uint8_t)(val) >> 8UL) & 0x0F)]) << 8UL) +  \
                                                   ((uint32_t)u4_to_hex_char[(((uint8_t)(val) >> 12UL) & 0x0F)]))

/// Макрос атрибута расположения данных в секции ramDMA
#ifndef __DMA_SECTION
  #define __DMA_SECTION __attribute__ ((section ("ramDMA")))
#endif /* #ifndef __DMA_SECTION */

/// Макрос упрощённой проверки на вхождение в интервал с явным заданием минимального и максимального значения
#define ISININTERVAL(val, min, max)               (((val) >= (min)) && ((val) <= (max)))
/*!
  \brief Макрос проверки значения переменной в окрестности заданной точки
  \param[in] val Проверяемое значение
  \param[in] bias Значение, в окрестности которого проводится проверка
  \param[in] range окрестности точки
*/
#define ISINRANGE(val, bias, range) ((((val) + (range)) >= (bias)) && \
                                     ((val) <= ((bias) + (range))))

/// Макрос патерна для 8 бит
#define BYTE_TO_BINARY_PATTERN                    "%c%c%c%c%c%c%c%c"
/// Макрос перевода 8-битного числа в строку бинарного представления
#define BYTE_TO_BINARY(word)  \
  (word & 0x80 ? '1' : '0'), \
  (word & 0x40 ? '1' : '0'), \
  (word & 0x20 ? '1' : '0'), \
  (word & 0x10 ? '1' : '0'), \
  (word & 0x08 ? '1' : '0'), \
  (word & 0x04 ? '1' : '0'), \
  (word & 0x02 ? '1' : '0'), \
  (word & 0x01 ? '1' : '0')

/// Макрос патерна для 32 бит
#define WORD_TO_BINARY_PATTERN                    "%c%c%c%c%c%c%c%c %c%c%c%c%c%c%c%c %c%c%c%c%c%c%c%c %c%c%c%c%c%c%c%c"

/// Макрос перевода 32-битного числа в строку бинарного представления
#define WORD_TO_BINARY(word)         \
    (word & 0x80 ? '1' : '0'),       \
    (word & 0x40 ? '1' : '0'),       \
    (word & 0x20 ? '1' : '0'),       \
    (word & 0x10 ? '1' : '0'),       \
    (word & 0x08 ? '1' : '0'),       \
    (word & 0x04 ? '1' : '0'),       \
    (word & 0x02 ? '1' : '0'),       \
    (word & 0x01 ? '1' : '0'),       \
    (word & 0x8000 ? '1' : '0'),     \
    (word & 0x4000 ? '1' : '0'),     \
    (word & 0x2000 ? '1' : '0'),     \
    (word & 0x1000 ? '1' : '0'),     \
    (word & 0x0800 ? '1' : '0'),     \
    (word & 0x0400 ? '1' : '0'),     \
    (word & 0x0200 ? '1' : '0'),     \
    (word & 0x0100 ? '1' : '0'),     \
    (word & 0x800000 ? '1' : '0'),   \
    (word & 0x400000 ? '1' : '0'),   \
    (word & 0x200000 ? '1' : '0'),   \
    (word & 0x100000 ? '1' : '0'),   \
    (word & 0x080000 ? '1' : '0'),   \
    (word & 0x040000 ? '1' : '0'),   \
    (word & 0x020000 ? '1' : '0'),   \
    (word & 0x010000 ? '1' : '0'),   \
    (word & 0x80000000 ? '1' : '0'), \
    (word & 0x40000000 ? '1' : '0'), \
    (word & 0x20000000 ? '1' : '0'), \
    (word & 0x10000000 ? '1' : '0'), \
    (word & 0x08000000 ? '1' : '0'), \
    (word & 0x04000000 ? '1' : '0'), \
    (word & 0x02000000 ? '1' : '0'), \
    (word & 0x01000000 ? '1' : '0')

/// Результат операции
typedef enum
{
  Done,                                                                                                                 ///< Операция выполнена успешно
  Error,                                                                                                                ///< Ошибка
  Busy                                                                                                                  ///< Ожидание выполнения предыдущей операции
} Result;

/// Состояние
typedef enum
{
  Disable = 0,                                                                                                          ///< Выключено
  Enable                                                                                                                ///< Включено
} State_t;

/// Структура битового поля знакового 16-битного числа TInt2x8
typedef struct {
  int16_t low :8;                                                                                                       ///< Младшие 8 бит 16битного знакового числа
  int16_t high :8;                                                                                                      ///< Старшие 8 бит 16битного знакового числа
} TInt2x8;

/// Структура битового поля беззнакового 16-битного числа TUnt2x8
typedef struct {
  uint16_t low :8;                                                                                                      ///< Младшие 8 бит 16битного беззнакового числа
  uint16_t high :8;                                                                                                     ///< Старшие 8 бит 16битного беззнакового числа
} TUint2x8;

/// Структура (объединение) знакового и безнакового 16-битного числа
typedef union {
  int8_t i8[2];
  int16_t i16;

  uint8_t u8[2];
  uint16_t u16;
} TInt16;

/// Структура (объединение) знакового и безнакового 32-битного числа
typedef union {
  int8_t i8[4];
  int16_t i16[2];
  int32_t i32;

  uint8_t u8[4];
  uint16_t u16[2];
  uint32_t u32;
} TInt32;

/// Структура (объединение) знакового и безнакового 64-битного числа
typedef union {
  int8_t i8[8];
  int16_t i16[4];
  int32_t i32[2];
  int64_t i64;

  uint8_t u8[8];
  uint16_t u16[4];
  uint32_t u32[2];
  uint64_t u64;
} TInt64;

/// Массив преобразования числа в hex символ
extern char const u4_to_hex_char[0x10];

/// Системная тактовая частота (частота ядра)
extern uint32_t SystemCoreClock;

/*!
  \brief Исполнительная функция при невыполнении заданного утверждения (условия)
  \note Вызывается в assert_func(). Необходимо реализовать в общей шаблонной части проекта
  \see assert_func(const char *file , int line)
  \param[in] module наименование модуля, в котором было вызвано исключение
  \param[in] file наименование файла, в котором было вызвано исключение
  \param[in] line значение текущей строки, в которой было вызвано исключение
  \param[in] function наименование функции, в которой было вызвано исключение
  \param[in] msg Диагностическое сообщение
 */
void safe_mode_loop(uint8_t module, const char *file, int line,
  const char *function, const char *msg);

/*!
  \brief Обработчик assert
  \param[in] module модуль где произошел assert
  \param[in] file файл где произошел assert
  \param[in] line строка в файле где произошел assert
  \param[in] function строка в файле где произошел assert
  \param[in] msg сообщения assert, может быть NULL
*/
void assert_func(uint8_t module, const char *file, int line, const char *function, const char *msg);

/*!
  \brief Проверка вхождения значения в заданный интервал
  \note Границы входят в интервал.
  \param[in] value заданное значение
  \param[in] bound1 значение одной границы интервала
  \param[in] bound2 значение второй границы интервала
  \retval 1: Да
  \retval 0: Нет
 */
static inline __attribute__((always_inline)) uint8_t IsInInterval(uint32_t const value, uint32_t const bound1, uint32_t const bound2)
{
  return ( ( (bound1 < bound2) && (value <= bound2) && (value >= bound1) ) ||
           ( (bound1 > bound2) && (value >= bound2) && (value <= bound1) ) ||
           ( (bound1 == bound2)&& (value == bound1) ) ) ? 1U : 0U;
}

/*!
  \brief Преобразование 8ми битного значения в hex строку
  \param[out] str указатель на буфер строки для сохранения hex символов
  \param[in] byte 8ми битное значения для преобразования
*/
static inline void u8_to_hex_str(char str[2], uint8_t byte)
{
  str[0] = u4_to_hex_char [byte >> 4];
  str[1] = u4_to_hex_char [byte & 0x0f];
}

/*!
  \brief Преобразование 16ти битного значения в hex строку
  \param[out] str указатель на буфер строки для сохранения hex символов
  \param[in] word 16ти битное значения для преобразования
*/
static inline void u16_to_hex_str(char str[4], uint16_t word)
{
  u8_to_hex_str(&str[0], (word >> 8) & 0xFF);
  u8_to_hex_str(&str[2], word & 0xFF);
}

/*!
  \brief Преобразование 32ти битного значения в hex строку
  \param[out] str указатель на буфер строки для сохранения hex символов
  \param[in] dword 32ти битное значения для преобразования
*/
static inline void u32_to_hex_str(char str[4], uint32_t dword)
{
  u16_to_hex_str(&str[0], (dword >> 16) & 0xFFFF);
  u16_to_hex_str(&str[4], dword & 0xFFFF);
}

/*!
  \brief Преобразование 8ми битного значения в десятичную строку
  \param[out] str указатель на буфер строки для сохранения hex символов
  \param[in] i 8ми битное значения для преобразования
*/
void u8_to_str(char str[3], uint8_t i);

/*!
  \brief Ограниченное сложение/вычитание
  \param[in,out] variable адрес изменяемого значения
  \param[in] conf условия 1/0 - инкремент/декремент
  \param[in] max Верхнее ограничение при ограниченном сложении/вычитании
  \param[in] min Нижнее ограничение при ограниченном сложении/вычитании
*/
void utils_limit_change_u8(uint8_t *variable, uint8_t conf, uint8_t max, uint8_t min);

///\brief Функция подсчета кол-ва единичных бит в 8-мибитном числе
///       Источник: https://stackoverflow.com/a/25808559
///\param bitset число для подсчета бит
///\return Кол-во единичных бит в числе
uint8_t calc_bits_8(uint8_t bitset);

///\brief Функция проверки четности 32-битного значения
///\param v Проверяемое значение
///\return  Бит четности
uint8_t calc_parity_32(uint32_t v);

/*!
  \}
*/
#ifdef __cplusplus
}
#endif /* #ifdef __cplusplus */
#endif /* #ifndef __UTILS_H_ */
