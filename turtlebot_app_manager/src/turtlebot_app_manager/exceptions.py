class AppException(Exception): 
    """
    Base exception class for App exceptions
    """
    pass

class InvalidAppException(AppException): 
    """
    App specification is invalid.
    """
    pass

class NotFoundException(AppException): 
    """
    Resource is not installed.
    """
    pass

class LaunchException(AppException): 
    """
    Exception thrown related to launching an App
    """
    pass

class InternalAppException(Exception): 
    """
    Base exception class for App exceptions
    """
    pass
