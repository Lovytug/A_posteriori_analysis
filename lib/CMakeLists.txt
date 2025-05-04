set(SRC_FILE
	Base/MonitoringComplex.cpp
	Ship/Ship.cpp 
	Helicopter/Helicopter.cpp 
	Base/Locator.cpp 
	Helicopter/OnBoardLocatorHelicopter.cpp
	Ship/OnBoardLocatorShip.cpp
	Base/KalmanFilter.cpp
	Helicopter/AimingSystem.cpp
	Global/RandomnessGenerator.cpp
	
)

add_library(MyLib STATIC ${SRC_FILE})

target_include_directories(MyLib PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/Base
	${CMAKE_CURRENT_SOURCE_DIR}/Ship
	${CMAKE_CURRENT_SOURCE_DIR}/Helicopter
	${CMAKE_CURRENT_SOURCE_DIR}/Global
	${eigen_SOURCE_DIR}
)

target_compile_features(MyLib PUBLIC cxx_std_17)