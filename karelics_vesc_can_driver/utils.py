

def get_current_time_seconds(node) -> float:
    current_time = node.get_clock().now().to_msg()
    current_time_sec = current_time.sec
    current_time_nanosec = current_time.nanosec
    return float(current_time_sec) + float(current_time_nanosec) * (10 ** -9)
