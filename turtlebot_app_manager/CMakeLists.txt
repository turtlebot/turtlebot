cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_app_manager)


find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(DIRECTORY msg
                    FILES
                    AppInstallationState.msg
                    AppList.msg
                    AppStatus.msg
                    ClientApp.msg
                    ExchangeApp.msg
                    Icon.msg
                    KeyValue.msg
                    StatusCodes.msg
                  )

add_service_files(DIRECTORY srv
                    FILES
                    GetAppDetails.srv
                    GetInstallationState.srv
                    InstallApp.srv
                    ListApps.srv
                    StartApp.srv
                    StopApp.srv
                    UninstallApp.srv
                 )


catkin_python_setup()

generate_messages()

catkin_package()
install(
    PROGRAMS
      scripts/app_manager
      scripts/appmaster
      scripts/rosget
      scripts/test_app.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#add_pyunit(test/test_app.py)
#add_pyunit(test/test_app_list.py)
