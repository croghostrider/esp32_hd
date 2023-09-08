# coding=cp1251
"""
������� ����������� � ���� ��� ����������� ���������� �����������/������������  esp32_HD 
�� REST API �������� ������ � ����������� � ������� ��������, ���������� � ����
���������:
ip - IP ����������� (���� �������� �� ������, ����� 192.168.0.44)
filename - ���� � ��� ����� ����. ���� �������� �� �����, ����� esp32_hd.log
-p - ������ ������ � ��������, ���� �������� �� ����� - 5 ���
--� - ���� ������� ����� ���� ����� �������. ���� �� �����, � ���� ���������� - �� ������������

������ ���������� ���������� �������� � ���������� LOG_FIELDS
"""
import csv, json, sys
import time
import argparse
import requests
from requests.exceptions import HTTPError

#LOG_FIELDS = ['uptime', 'MainMode', 'MainStatus', 'CurVolts', 'CurPower', 'SetPower', 'AlarmMode','sensors','klapans']
LOG_FIELDS = [ 'time', 'uptime', 'MainMode', 'MainStatus', 'waitStr', 'CurPower', 'SetPower', 'Alarm','sensors','klapans','adc','bmpTruePressure','bmpTemperature']

def openFile(filename, clr):
    try:
        return open(filename, "w" if clr!=0 else "a")
    except Exception as e:
        sys.stderr.write(f'could not open file {filename}: {e}\n')
        return None

def readHttp(ip):
    try:
        response = requests.get(f'http://{ip}/maininfo')
        response.raise_for_status()
    except HTTPError as http_err:
        print(f'HTTP error occurred: {http_err}')  # Python 3.6
    except Exception as err:
        print(f'Other error occurred: {err}')  # Python 3.6
    return response


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='esp32HD logger')
    parser.add_argument(
        "filename",
        nargs='?',
        help="filename of log [esp32_hd.log by default]",
        default="esp32_hd.log")
    parser.add_argument(
        '-ip',
        nargs='?',
        help="IP esp32_hd",
        default=None)
    parser.add_argument('--c', action='store_const', help="clear file", const = 0, default = 0)
    parser.add_argument("-period", "-p", nargs='?', type=int, help="logging period,sec [5 sec by default]",default =5)
    args = parser.parse_args()

    if not args.period:
        args.period = 5

    print(f"IP:{args.ip} file:{args.filename} period:{args.period} sec  cleared:{args.c}")

    if args.ip is None:
        parser.error('IP must be specifyied')
        sys.exit(0)

    if args.filename is None:
        parser.error('filename  must be specifyied')
        sys.exit(0)

    if (not readHttp(args.ip)):
        sys.exit(0)
    outputFile = openFile(args.filename,args.c)
    head_printed = 0
    try:
        while True:
            r = readHttp(args.ip)
            if (not r): continue
            data = r.json()
            if head_printed==0:
                s = ''
                for i in LOG_FIELDS:
                    if i=='sensors':
                        for s0 in data.get(i):
                            s = f"{s}T{s0.get('type_str')};"
                                                    #outputFile.write(f"T{s0.get('type_str')};")
                    elif i=='klapans':
                        for k0 in data.get(i):
                            s = f"{s}V{k0.get('id')}open;"
                            if k0.get('id') == 2:
                                s = f"{s}V{k0.get('id')}pwm;V{k0.get('id')}period;V{k0.get('id')}prc;"
                                                        #outputFile.write(f"V{k0.get('id')};")
                    elif i=='adc':
                        try:
                            for a0 in data.get(i):
                                s = f"{s}A{a0.get('id')}val;"
                        except Exception:
                            pass
                    else:
                        s = f"{s}{i};"
                print(s)
                outputFile.write(s)
                outputFile.write('\n')
                head_printed = 1
            print(f"{data.get('uptime')}: {data.get('MainModeStr')}/{data.get('MainStatusStr')}")
            for x in LOG_FIELDS:
                if x=='sensors':
                    for s in data.get(x):
                        outputFile.write(f"{s.get('temp')};")
                elif x=='klapans':
                    for k in data.get(x):
                        outputFile.write(f"{k.get('is_open')};")
                        if k.get('id')==2:
                            outputFile.write(f"{k.get('is_pwm')};{k.get('pwm_time')};{k.get('pwm_percent')};")
                elif x=='adc':
                    try:
                        for a in data.get(x):
                            outputFile.write(f"{a.get('val')};")
                    except Exception:
                        pass
                else:
                    v = data.get(x)
                    outputFile.write(f"{v};")
            outputFile.write('\n')
            outputFile.flush()
            time.sleep(args.period)
    except Exception as e:
      outputFile.close()
      raise e
    outputFile.close()
    print('end of work')
