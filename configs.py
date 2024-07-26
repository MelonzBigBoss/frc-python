from phoenix6 import configs, signals

class swerve:
    cancoder = configs.CANcoderConfiguration()
    cancoder.magnet_sensor.sensor_direction = signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE

    angle = configs.TalonFXConfiguration()

    drive = configs.TalonFXConfiguration()