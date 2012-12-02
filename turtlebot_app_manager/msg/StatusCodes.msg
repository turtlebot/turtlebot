# Common error codes used with App Manager.
int32 SUCCESS = 0
# Request was invalid.
int32 BAD_REQUEST = 400
# App is not installed.
int32 NOT_FOUND = 404
# App is not running.
int32 NOT_RUNNING = 430
# Unknown internal error on the server.
int32 INTERNAL_ERROR = 500
# App is installed but failed validation.
int32 APP_INVALID = 510
# App manager does not support launching multiple apps simultaneously. Running app must first be stopped.
int32 MULTIAPP_NOT_SUPPORTED = 511
