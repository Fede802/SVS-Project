import os

def get_last_log_file(log_dir = "logs"):
    if not os.path.exists(log_dir):
        return 0
    files = os.listdir(log_dir)
    max_index = 0
    max_index_file = ""
    for f in files:
        log_number = int(f.split("_")[-1].split(".")[0])
        if log_number > max_index:
            max_index_file = f
            max_index = log_number
    return max_index_file

class Logger:
    def __init__(self, log_dir = "logs", log_file_name_prefix = "log"):
        self.log_file = open(self.__compute_file_path(log_dir, log_file_name_prefix), "w")
    
    def __compute_file_path(self, log_dir, log_file_name_prefix):
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        last_log_filename = get_last_log_file(log_dir)
        log_number = int(last_log_filename.split("_")[-1].split(".")[0]) if last_log_filename != "" else 0
        return log_dir+"/"+log_file_name_prefix + "_" + str(log_number + 1) + ".txt"    

    def write(self, message):
        self.log_file.write(message + "\n")
        
    def close(self):
        self.log_file.close()    