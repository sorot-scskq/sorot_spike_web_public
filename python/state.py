import threading

DEFAULT_RESULT = {
    "completed_req_id": 0,
    "minifigure": 100,
    "plarail": 100,
    "bottle": 100,
    "gate": 100,
    "qr1": "",
    "qr2": ""
}

latest_result = dict(DEFAULT_RESULT)
result_lock = threading.Lock()
pc_lock = threading.Lock()
