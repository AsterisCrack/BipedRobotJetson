"""ST3215 register map and instruction set (SCS/Feetech protocol)."""


class Reg:
    # --- EEPROM (persists across power cycles) ---
    FIRMWARE_MAJOR   = 0x00
    FIRMWARE_MINOR   = 0x01
    SERVO_MAJOR      = 0x03
    SERVO_MINOR      = 0x04
    ID               = 0x05
    BAUD_RATE        = 0x06
    RETURN_DELAY     = 0x07
    RESPONSE_LEVEL   = 0x08
    MIN_ANGLE_L      = 0x09   # 2 bytes
    MAX_ANGLE_L      = 0x0B   # 2 bytes
    MAX_TEMP         = 0x0D
    MAX_VOLTAGE      = 0x0E   # unit: 0.1 V
    MIN_VOLTAGE      = 0x0F   # unit: 0.1 V
    MAX_TORQUE_L     = 0x10   # 2 bytes, 0-1000 (100%)
    PHASE            = 0x12
    UNLOAD_COND      = 0x13
    LED_ALARM        = 0x14
    PID_P            = 0x15
    PID_D            = 0x16
    PID_I            = 0x17
    MIN_STARTUP_L    = 0x18   # 2 bytes
    CW_DEADBAND      = 0x1A
    CCW_DEADBAND     = 0x1B
    PROT_CURRENT_L   = 0x1C   # 2 bytes, unit: 6.5 mA
    ANGLE_RESOLUTION = 0x1E
    POS_CORRECTION_L = 0x1F   # 2 bytes, signed (bit11 = sign)
    OPERATION_MODE   = 0x21
    PROT_TORQUE      = 0x22   # % of max
    PROT_TIME        = 0x23   # unit: 10 ms
    OVERLOAD_TORQUE  = 0x24   # %
    SPEED_P          = 0x25
    OVERCURRENT_TIME = 0x26
    SPEED_I          = 0x27

    # --- RAM (lost on power cycle) ---
    TORQUE_ENABLE    = 0x28   # 0=off, 1=on, 128=zero-calibrate
    ACCELERATION     = 0x29   # unit: 100 step/s²
    TARGET_POS_L     = 0x2A   # 2 bytes, little-endian, 0-4095
    RUN_TIME_L       = 0x2C   # 2 bytes
    TARGET_SPEED_L   = 0x2E   # 2 bytes, step/s
    TORQUE_LIMIT_L   = 0x30   # 2 bytes
    LOCK_FLAG        = 0x37

    # --- RAM read-only status ---
    CURRENT_POS_L    = 0x38   # 2 bytes
    CURRENT_SPEED_L  = 0x3A   # 2 bytes
    CURRENT_LOAD_L   = 0x3C   # 2 bytes
    CURRENT_VOLTAGE  = 0x3E   # unit: 0.1 V
    CURRENT_TEMP     = 0x3F   # °C
    ASYNC_WRITE_FLAG = 0x40
    SERVO_STATUS     = 0x41
    MOVE_FLAG        = 0x42
    CURRENT_CURRENT_L= 0x45   # 2 bytes, unit: 6.5 mA

    # Bulk status read: 8 bytes from 0x38 → pos, speed, load
    STATUS_START     = 0x38
    STATUS_LEN       = 8      # pos(2) + speed(2) + load(2) + voltage(1) + temp(1)


class Instr:
    PING       = 0x01
    READ       = 0x02
    WRITE      = 0x03
    REG_WRITE  = 0x04   # buffer write; executes on ACTION
    ACTION     = 0x05   # execute all buffered REG_WRITEs simultaneously
    RESET      = 0x06
    SYNC_READ  = 0x82   # broadcast read from multiple servos; each responds in turn
    SYNC_WRITE = 0x83   # broadcast write to multiple servos in one packet


# Baud rate index → actual baud rate
BAUD_RATES = {
    0: 1_000_000,
    1: 500_000,
    2: 250_000,
    3: 128_000,
    4: 115_200,
    5: 76_800,
    6: 57_600,
    7: 38_400,
}
