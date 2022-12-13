from multiprocessing.dummy import Process


def f1():
    i = 0
    while i < 500:
        print('b')
        i += 1


def f2():
    i = 0
    while i < 100:
        print('a')
        i += 1

p1 = Process(target=f1, args=())
p2 = Process(target=f2, args=())

p1.start()
p2.start()

p1.join()
p2.join()