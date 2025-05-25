extends Node

class_name PIDController

var kp: float
var ki: float
var kd: float

var setpoint: float = 0.0
var input: float = 0.0
var output: float = 0.0

var prev_error: float = 0.0
var integral: float = 0.0
var last_time: float = 0.0

func _init(kp_: float, ki_: float, kd_: float):
	kp = kp_
	ki = ki_
	kd = kd_
	last_time = Time.get_ticks_msec() / 1000.0

func set_constants(new_kp: float, new_ki: float, new_kd: float) -> void:
	kp = new_kp
	ki = new_ki
	kd = new_kd

func compute():
	var current_time = Time.get_ticks_msec() / 1000.0
	var delta_time = current_time - last_time
	if delta_time <= 0:
		return
	
	var error = setpoint - input
	integral += error * delta_time
	var derivative = (error - prev_error) / delta_time if delta_time > 0 else 0.0
	
	output = (kp * error) + (ki * integral) + (kd * derivative)

	prev_error = error
	last_time = current_time
