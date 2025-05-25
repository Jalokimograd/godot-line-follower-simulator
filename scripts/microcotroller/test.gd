extends Node

@onready var ACTOR = get_parent()

# Parametry konfiguracyjne
@export var HISTORY_SIZE 	= 10        # Liczba zapamiętanych pomiarów (dla analizy historii)
@export var NO_LINE_COUNT  	= 20        # Liczba kroków, po których uznajemy, że linia zaginęła
@export var SENSOR_COUNT 	= 7         

# Ograniczenia prędkości silników
var left_min_speed 		= -50
var right_min_speed 	= -50
var left_max_speed 		= 255
var right_max_speed 	= 255

# Dane z czujników i historia odczytów
var sensor_values = []
var reading_history = []

# Wagi dla każdego czujnika
var sensor_weights = [	
	12.5, 
	6.0,
	0.5, 
	0.1, 
	0.5,
	6.0, 
	12.5
]

# Parametry PID
@export var Kp = 1
@export var Ki = 0.01
@export var Kd = 0.01

# Inicjalizacja kontrolerów PID dla obu silników
@onready var pid_left 	= PIDController.new(Kp, Ki, Kd)
@onready var pid_right 	= PIDController.new(Kp, Ki, Kd)

# Zmienne do śledzenia błędu linii i trybu awaryjnego
var line_error: float = 0.0
var line_threshold: float = 0.4
var no_line_count: int = 0
var last_known_direction: int = 0  # Kierunek, w którym ostatnio była linia

# Odczyt danych z czujników i obliczanie błędu pozycji względem linii
func read_and_calculate_error():
	var line_detected = false
	line_error = 0.0

	for i in range(SENSOR_COUNT):
		var sensor_val = ACTOR.get_sensor_value(i)
		# Obliczanie błędu jako ważona suma różnic od środka czujników
		line_error += (i - (SENSOR_COUNT - 1) / 2.0) * (1 - sensor_val) * sensor_weights[i]
		if sensor_val < line_threshold:
			line_detected = true

	# Jeśli nie wykryto linii
	if not line_detected:
		no_line_count += 1
		if no_line_count > NO_LINE_COUNT:
			# Wchodzimy w tryb awaryjny — szukamy kierunku ostatniej linii
			line_error = find_where_last_line_went()
	else:
		no_line_count = 0
		# Zapamiętujemy ostatni kierunek, w którym była linia
		last_known_direction = sign(line_error)

# Obliczanie przybliżonego kierunku linii na podstawie historii czujników
func find_where_last_line_went() -> int:
	var history_error = 0.0

	for l in range(HISTORY_SIZE - 1, -1, -1):
		for i in range(SENSOR_COUNT):
			# Analiza historycznych danych czujników z zależną wagą
			history_error += (i - (SENSOR_COUNT - 1) / 2.0) * ACTOR.get_sensor_value(i) * sensor_weights[i] * (l / 10.0)

	# Zwraca kierunek: 1 (prawo), -1 (lewo), 0 (brak pewności)
	return 1 if history_error > 50 else -1 if history_error < -50 else 0

# Obliczanie prędkości silników z użyciem PID na podstawie błędu
func calculate_motor_speed():
	pid_left.input 	= clampf(line_error, -20, 20) / 20.0 * 255
	pid_right.input = clampf(-line_error, -20, 20) / 20.0 * 255
	pid_left.compute()
	pid_right.compute()

# Dynamiczne skręcanie na podstawie błędu
func set_motor_speed():
	var base_speed = 150
	var max_correction = 200

	# Korekta prędkości zależna od błędu
	var correction = clamp(line_error / 20.0, -1.0, 1.0) * max_correction

	# Obliczanie docelowych prędkości silników
	var left_speed = base_speed - correction
	var right_speed = base_speed + correction

	# Ustawienie prędkości silników
	ACTOR.left_motor.set_velocity(clamp(left_speed, left_min_speed, left_max_speed))
	ACTOR.right_motor.set_velocity(clamp(right_speed, right_min_speed, right_max_speed))

# Punkt 2: Tryb awaryjny — obrót w miejscu w celu odnalezienia linii
func execute_emergency_turn():
	var turn_speed = 100
	if last_known_direction >= 0:
		# Obrót w prawo
		ACTOR.left_motor.set_velocity(turn_speed)
		ACTOR.right_motor.set_velocity(-turn_speed)
	else:
		# Obrót w lewo
		ACTOR.left_motor.set_velocity(-turn_speed)
		ACTOR.right_motor.set_velocity(turn_speed)

# Inicjalizacja struktur danych
func _ready() -> void:
	sensor_values.resize(SENSOR_COUNT)
	reading_history.resize(SENSOR_COUNT)
	for i in range(SENSOR_COUNT):
		reading_history[i] = []
		reading_history[i].resize(HISTORY_SIZE)

# Główna pętla działania — wykonywana co klatkę
func _process(delta: float) -> void:
	read_and_calculate_error()

	if no_line_count > NO_LINE_COUNT:
		# Tryb awaryjny — robot zgubił linię
		execute_emergency_turn()
	else:
		# Normalna jazda z regulacją PID
		calculate_motor_speed()
		set_motor_speed()
