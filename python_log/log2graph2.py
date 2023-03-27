# coding=cp1251
'''
отображает графики температур из лога 
показывает 3 окна
1е и 3е - полный лог, по всем строкам файла
2е окно показывает  данные последних 80 (или заданых параметром -t) строк лога 

параметры запуска:
log2graph.py {-p[N]} {-t[N]} {filename}
-p[N]  - период в сек. перечитывания файла лога и обновления графиков. Если не указан - период будет 2 сек
-p0 - выключит перечитывания/обновление. Используется при изучении старых записаных логов
-t - количество строк лога показываемых во втором графике. Минимум 2. По умолчанию 80 (константа TAIL_LEN)
-h - help, покажет опции и формат запуска
filename - имя файла. Если не задан, то используется esp32hd.log

Примеры запуска:
>log2graph.py
читает файл esp32hd.log с периодом 2 секунды, если файл меняется - будет обновлять графики

>log2graph.py мойЛог.log
будет читать файл мойЛог.log с периодом 2 секунды, если файл меняется - будет обновлять графики

>log2graph.py -p15 мойЛог.log
будет читать файл мойЛог.log с периодом 15 секунд

>log2graph.py -p15 -t100 мойЛог.log
будет читать файл мойЛог.log с периодом 15 секунд, второй график будет показывать данные 100 последних строк лога

>log2graph.py -p0 -t99999999999 мойСтарыйЛог.log
однократно будет считан и показан лог из файла мойСтарыйЛог.log. Если файл изменяется - графики меняться не будут
второй график будет показывать полный лог, если его длина меньше 99999999999 строк

===========================================================
Формат лога: csv , разделитель ';'
в первой строке д.быть имена полей лога, разделитель ';'

(!!!)ПЕРВЫМ полем в логе дбыть uptime - время в формате HH:MI:SS от начала процесса

Пример лога:
time;uptime;MainMode;MainStatus;waitStr;CurPower;SetPower;Alarm;V0open;V1open;V2open;V2pwm;V2period;V2prc;V3open;A0val;A1val;A2val;A3val;bmpTruePressure;bmpTemperature;
10:29;00:24:17;2;0;;-1;0;128;0;0;0;0;0;0;0;172;4798;4810;4822;741.61;24.2;
10:30;00:24:22;2;0;;-1;0;128;0;0;0;0;0;0;0;178;4788;4820;4842;741.64;24.2;
10:30;00:24:27;2;0;;-1;0;128;0;0;0;0;0;0;0;172;4798;4830;4834;741.61;24.3;
...

какие поля выбирать для графика задает константа SENSORS_NAME
Какие поля лога отображать в графиках задает константа GRAPH_LOGS
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
from datetime import datetime

#список идентификаторов полей лога из которых берутся данные для графиков
 
SENSORS_NAME = "TCube;TTube 20%;TTube;TDeflegmator;TWater IN;TWater Out;TAlarm;".split(';')
#time;uptime;MainMode;MainStatus;waitStr;CurPower;SetPower;Alarm;TCube;TTube 20%;TTube;TDeflegmator;TWater IN;TWater Out;TAlarm;V0open;V1open;V2open;V2pwm;V2period;V2prc;V3open;bmpTruePressure;bmpTemperature;

# задаем какие переменные из списка выше в каком окне отображать. 
# первый массив - первое окно, второй - второе, третий - третье
# число в массиве -  индекс переменной  в предыдущем списке
#GRAPH_LOGS  = [[0,1,2,4],[1,2,4],[5,6,7]]
GRAPH_LOGS  = [[0,1,2,3],[1,2,3],[4,5,6]]

TAIL_LEN = 160
PERIOD_SEC = 0

pos_sensor = []
period = PERIOD_SEC
tail_len = TAIL_LEN
axeXfield_name = 'uptime'
uptime_index = 1

def read_header(fname):
    pos_sensor = []
    arr = []
    try:
        with open(fname, "r", newline="") as file:
          #читаем файл целиком
          reader = csv.reader(file)
          line_no=0
          for row in reader:
            #import ipdb; ipdb.set_trace()
            if row:
                fields = row[0].split(';')
                if (fields.count(axeXfield_name)==0): #игнорим строки где нет имен полей 
                    continue
                for sname in SENSORS_NAME:
                    arr.append([])
                    if sname in fields:
                        pos_sensor.append(fields.index(sname))
                    else:
                        pos_sensor.append(-1)
                return pos_sensor
    except Exception:
        print(f'ошибка чтения заголовка файла {fname}')
        return []

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
        
def readCsv(fname):
    if len(pos_sensor)==0:
        return []
    
    # создаем список массивов для отображения графиков сенсоров + 1 на ось X
    arr = []
    for i in range(len(pos_sensor)+1): 
      arr.append([])
    #читаем данные из файла
    nrow=1
    with open(fname, "r", newline="") as file:
        #читаем файл целиком
        reader = csv.reader(file)
        for row in reader:
            if row:
                nrow += 1
                # преобразуем строку в список
                fields = row[0].split(';')
                if (fields.count('time')): #заголовок
                    continue
                
                #пытаемся получить uptime в секундах. При ошибке получим None
                uptime = getSecUptime(fields[uptime_index])
                if (uptime==None):   #если uptime не вычисляется - на сл.строку
                    #print(f'ошибка разбора поля uptime "{fields[0]}"')
                    continue

                #добавим uptime в массив X
                arr[len(pos_sensor)].append(uptime)
                #выбираем из строки поля с данными датчиков и добавляем их в соответствующие массивы Y
                for i in range(len(pos_sensor)): 
                    if pos_sensor[i] == -1: # если сенсора нет в заголовке файла
                        continue
                    # преобразовываем поле в число
                    try:
                        v = float(fields[pos_sensor[i]])
                    except ValueError:
                        v = None # если ошибка преобразования в  число
                    #добавляем в массив значение
                    arr[i].append(v)
    return arr

def getTailStart(arr):
    ret = len(arr[0]) - tail_len
    if ret <0:
        ret = 0
    return ret

def plot_cont(filepath):
    
    from pylab import rcParams        
    rcParams['figure.subplot.left'] = 0.04 # левая граница
    rcParams['figure.subplot.right'] = 0.96 # правая граница
    rcParams['figure.subplot.bottom'] = 0.1 # нижняя граница
    rcParams['figure.subplot.top'] = 0.95 # верхняя граница
    
    fig = plt.figure()
    fig.set_size_inches(16,6)
    ax  = fig.add_subplot(1,3,1)
    ax2 = fig.add_subplot(1,3,2)
    ax3 = fig.add_subplot(1,3,3)

    def update(i):
        global pos_sensor
        arr = readCsv(filepath)
        if len(arr)==0:
            return 
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
        ax.set_xlabel('Uptime, сек')
        ax.xaxis.set_major_formatter( DateFormatter('%H:%M:%S') )
        #ax.set_xticks(df.index)
        #ax.set_xticklabels(df.manufacturer.str.upper(), rotation=60, fontdict={'horizontalalignment': 'right', 'size':12})
        #ax.set_ylabel('Температура, С')
        #ax.set_title ('Полный график')
        
        #-------------------------------------
        tail_start = getTailStart(arr)
        xlist2 = xlist[tail_start:]
        ax2.clear()
        c = 0
        for i in GRAPH_LOGS[1]:
          if pos_sensor[i]>0:
            ax2.plot (xlist2, arr[i][tail_start:], color[c], label = SENSORS_NAME[i])
            #print(arr[i][tail_start:])
          c+=1
        ax2.grid(True)
        ax2.legend ()
        ax2.set_xlabel('Uptime, сек')
        ax2.xaxis.set_major_formatter( DateFormatter('%H:%M:%S') )
        #ax2.set_ylabel('Tемпература,С')
        ax2.set_title(f'{len(xlist2)} отсчетов')
        #-------------------------------------
        ax3.clear()
        c = 0
        for i in GRAPH_LOGS[2]:
          if pos_sensor[i]>0:
            ax3.plot (xlist, arr[i], color[c], label = SENSORS_NAME[i])
          c+=1
        ax3.grid(True)
        ax3.legend ()
        ax3.set_xlabel('Uptime, сек')
        ax3.xaxis.set_major_formatter( DateFormatter('%H:%M:%S') )
        ax3.set_ylabel('T,С')
        ax3.set_title('Вода')
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
    parser.add_argument("-tail", "-t", nargs='?', type=int, help=f'point number of the 2d plot, min 2. By default {TAIL_LEN}', default=TAIL_LEN)
    args = parser.parse_args()

    period = args.period
    filepath = args.filename
    tail_len = args.tail
    if tail_len<2:
      tail_len = 2
    try:
        f = open(filepath)
        f.close
    except FileNotFoundError:
        print(f'Файл {filepath} не найден')
        sys.exit(0)

    print(f"logfile:{filepath} re-draw period:{period} sec tail:{tail_len}")
    pos_sensor = read_header(filepath)
    if ((pos_sensor.count(-1)>10) or (len(pos_sensor)==0)):
        print(f'Датчики температуры в логе "{filepath}"не найдены')
        sys.exit(0)
    plot_cont(filepath)
