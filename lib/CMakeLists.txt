set(SRC_FILE
	src/MonitoringComplex.cpp
	src/Ship.cpp 
	src/Helicopter.cpp 
	src/Locator.cpp 
	src/OnBoardLocatorHelicopter.cpp
	src/OnBoardLocatorShip.cpp
	src/KalmanFilter.cpp
	src/AimingSystem.cpp
	src/RandomnessGenerator.cpp
	
)

add_library(MyLib STATIC ${SRC_FILE})

target_include_directories(MyLib PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${eigen_SOURCE_DIR}
)

target_compile_features(MyLib PUBLIC cxx_std_17)