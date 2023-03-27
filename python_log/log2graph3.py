# coding=cp1251
'''
отображает графики ADC из лога 
показывает 1 окно с двумя графиками (проводимость тела и дефа)

параметры запуска:
log2graph.py {-p[N]} {-t[N]} {filename}
-p[N]  - период в сек. перечитывания файла лога и обновления графиков. Если не указан - период будет 2 сек
-p0 - выключит перечитывания/обновление. Используется при изучении старых записаных логов
-h - help, покажет опции и формат запуска
filename - имя файла. Если не задан, то используется esp32hd.log

Примеры запуска:
>log2graph3.py
читает файл esp32hd.log с периодом 2 секунды, если файл меняется - будет обновлять графики

>log2graph3.py мойЛог.log
будет читать файл мойЛог.log с периодом 2 секунды, если файл меняется - будет обновлять графики

>log2graph3.py -p15 мойЛог.log
будет читать файл мойЛог.log с периодом 15 секунд
===========================================================
Формат лога: csv , разделитель ';'
в первой строке д.быть имена полей лога, разделитель ';'

Пример лога:
time;uptime;MainMode;MainStatus;waitStr;CurPower;SetPower;Alarm;V0open;V1open;V2open;V2pwm;V2period;V2prc;V3open;A0val;A1val;A2val;A3val;bmpTruePressure;bmpTemperature;
10:29;00:24:17;2;0;;-1;0;128;0;0;0;0;0;0;0;172;4798;4810;4822;741.61;24.2;
10:30;00:24:22;2;0;;-1;0;128;0;0;0;0;0;0;0;178;4788;4820;4842;741.64;24.2;
10:30;00:24:27;2;0;;-1;0;128;0;0;0;0;0;0;0;172;4798;4830;4834;741.61;24.3;
'''
import csv
import sys
import argparse
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as anim
from matplotlib import mlab
import datetime, time
import matplotlib.dates as mdates
from matplotlib.dates import DateFormatter
import logutil

#список идентификаторов полей лога из которых берутся данные для графиков
SENSORS_NAME = "A0val;A1val;A2val;A3val;".split(';')

# задаем какие переменные из списка выше в каком окне отображать. 
# первый массив - первое окно, второй - второе, третий - третье
# число в массиве -  индекс переменной  в предыдущем списке
GRAPH_LOGS  = [[0,1],[2,3]]
PERIOD_SEC = 5
pos_sensor = []
period = PERIOD_SEC
axeXfield_name = 'uptime'
uptime_index = 1

       
def readCsv(fname):
    # создаем список массивов для отображения графиков сенсоров + 1 на ось X
    arr = []
    for i in range(len(pos_sensor)+1): 
      arr.append([])
    
    #читаем данные из файла
    with open(fname, "r", newline="") as file:
        #читаем файл целиком
        reader = csv.reader(file)
        for row in reader:
            if row:
                # преобразуем строку в список
                fields = row[0].split(';')
                uptime = logutil.getSecUptime(fields[uptime_index]) #пытаемся прочитать поле uptime
                if (uptime==None): #если uptime не вычисляется - на сл.строку
                    #print(f'ошибка разбора поля uptime "{fields[1]}"')
                    continue
                #добавим uptime в массив X
                arr[len(pos_sensor)].append(uptime)
                #ищем в строке поля датчиков и добавляем их в соответствующие массивы Y
                for i in range(len(pos_sensor)): # для всех сенсоров
                    if pos_sensor[i] == -1: # если сенсора нет в заголовке файла
                        continue
                    v = None
                    # преобразовываем в число
                    if fields[pos_sensor[i]].isdigit():
                        v = int(fields[pos_sensor[i]])
                    #добавляем в массив значение
                    arr[i].append(v)
    return arr

def plot_cont(filepath):
    from pylab import rcParams        
    rcParams['figure.subplot.left'] = 0.04 # левая граница
    rcParams['figure.subplot.right'] = 0.96 # правая граница
    rcParams['figure.subplot.bottom'] = 0.1 # нижняя граница
    rcParams['figure.subplot.top'] = 0.95 # верхняя граница
    
    fig = plt.figure()
    fig.set_size_inches(16,6)
    ax  = fig.add_subplot(1,2,1)
    ax3 = fig.add_subplot(1,2,2)

    def update(i):
        global pos_sensor
        arr = readCsv(filepath)
        xlist = arr[len(arr)-1]
        ax.clear()
        #SENSORS_NAME = "TCube;TTube 20%;TTube 80%;TDeflegmator;TTSA;TWater IN;TWater Out".split(';')
        color = ['b-','g--','r-','y--','c-','m-', 'k-', 'w-']
        c = 0
        for i in GRAPH_LOGS[0]:
            if pos_sensor[i]>0:
                ax.plot_date (xlist, arr[i], color[c], label = SENSORS_NAME[i])
            c+=1
        ax.grid(True)
        ax.legend ()
        ax.set_xlabel('Uptime hh:mm:sec')
        ax.xaxis.set_major_formatter( DateFormatter('%H:%M:%S') )
        ax.set_title ('ADC channel 0,1')
        
        #-------------------------------------
        ax3.clear()
        c = 0
        for i in GRAPH_LOGS[1]:
          if pos_sensor[i]>0:
            ax3.plot (xlist, arr[i], color[c], label = SENSORS_NAME[i])
          c+=1
        ax3.grid(True)
        #ax3.legend ()
        ax3.set_xlabel('Uptime hh:mm:sec')
        ax3.xaxis.set_major_formatter( DateFormatter('%H:%M:%S') )
        ax3.set_title ('ADC channel 2,3')
        fig.autofmt_xdate(rotation=45)
        '''
        # Plot Line2 (Right Y Axis)
        ax4 = ax3.twinx()  # instantiate a second axes that shares the same x-axis
        ax4.plot(x, y2, color='tab:blue')
        '''
    if period >0:
        a = anim.FuncAnimation(fig, update, interval=period*1000, repeat=True)
    else:
        update(1)
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='log2graph')
    parser.add_argument(
        "filename",
        nargs='?',
        help="filename of log [esp32_hd.log by default]",
        default="esp32_hd.log")
    parser.add_argument("-period", "-p", nargs='?', type=int, help=f"re-draw  period,sec.0-no redraw.  By default {PERIOD_SEC}. ", default=PERIOD_SEC)
    args = parser.parse_args()

    period = args.period
    filepath = args.filename
    try:
        f = open(filepath)
        f.close
    except FileNotFoundError:
        print(f'Файл {filepath} не найден')
        sys.exit(0)

    print(f"logfile:{filepath} re-draw period:{period} sec")

    header = logutil.get_header(filepath, axeXfield_name)
    if header==None:
        print(f"в файле '{filepath}' не найден заголовок")
        sys.exit(0)
        
    pos_sensor = logutil.findSensor(header, SENSORS_NAME)
    if ((pos_sensor.count(-1)>=len(SENSORS_NAME)) or (len(pos_sensor)==0)):
        print(f'отсчеты АЦП в логе "{filepath}"не найдены')
        sys.exit(0)
        
    plot_cont(filepath)
