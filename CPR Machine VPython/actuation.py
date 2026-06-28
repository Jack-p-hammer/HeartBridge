# actuation.py

from error_codes import ErrorCode

def init_motor() -> ErrorCode:
    # TODO: moteus CAN init
    return ErrorCode.NORMAL_OPERATION

def zeroing() -> ErrorCode:
    return ErrorCode.NORMAL_OPERATION

def stop_zeroing() -> ErrorCode:
    return ErrorCode.NORMAL_OPERATION

def init_compressions() -> ErrorCode:
    return ErrorCode.NORMAL_OPERATION

def compressions() -> ErrorCode:
    return ErrorCode.NORMAL_OPERATION

def stop_compressions() -> ErrorCode:
    return ErrorCode.NORMAL_OPERATION