# coding=cp1251
def get_header(filename, key_field_name):
    try:
        with open(filename, "r", newline="") as file:
            line_no=0
            for line in file:
                header = line.split(';')
                if (header.count(key_field_name)>0): #если в строке есть ключевое поле
                    return header; # это заголовок
    except Exception:
        print(f'ошибка чтения файла {filename}')
    return None;  

def  findSensor(header, sensor_name_list):
    ret_list = []
    for sensor in sensor_name_list:
        try:
            index =header.index(sensor)
        except Exception:
            index = -1
        ret_list.append(index)
    return ret_list

from datetime import datetime
def getSecUptime(uptime_str):
    # uptime_str д.быть в формате HH24:MI:SS
    try:
        if len(uptime_str.split(':'))==2:
            return datetime.strptime(uptime_str,'%M:%S')
        else:
            return datetime.strptime(uptime_str,'%H:%M:%S')
    except ValueError:
        #print(f'error uptime "{uptime_str}"')
        pass
    return None
