set(SRC_FILE
	src/MonitoringComplex.cpp
	src/Ship.cpp 
	src/Helicopter.cpp 
	src/Locator.cpp 
	src/KalmanFilter.cpp
	src/AimingSystem.cpp
)

add_library(MyLib STATIC ${SRC_FILE})

target_include_directories(MyLib PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(MyLib PUBLIC cxx_std_17)