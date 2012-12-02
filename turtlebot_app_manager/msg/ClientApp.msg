# like "android" or "web" or "linux"
string client_type

# like "intent = ros.android.teleop" and "accelerometer = true", used to choose which ClientApp to use
KeyValue[] manager_data

# parameters which just get passed through to the client app.
KeyValue[] app_data
