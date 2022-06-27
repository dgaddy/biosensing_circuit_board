import serial
import time
import numpy as np
import scipy.signal
import multiprocessing
import queue

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

n_channels = 8

class IncrementalDataFilter(object):
    def __init__(self, n_channels, max_len, sample_frequency):
        self.max_len = max_len
        self.sample_frequency = sample_frequency

        self.notch_zi = [np.zeros([2, n_channels]) for _ in range(60, int(sample_frequency/2), 60)]
        self.notch_zi_2 = [np.zeros([2, n_channels]) for _ in range(60, int(sample_frequency/2), 60)]
        self.highpass_zi = np.zeros([3, n_channels])

        self.processed_signals = np.zeros([max_len, n_channels])
        self.raw_processed_signals = np.zeros([max_len, n_channels])
        self.new_samples = []

        self.t_values = [0] * max_len

    def add_sample(self, sample, t_value=None):
        self.new_samples.append(sample)
        if t_value is not None:
            self.t_values.append(t_value)

    def get_signals(self):
        if len(self.new_samples) > 0:
            new_signals = np.array(self.new_samples)

            self.raw_processed_signals = np.concatenate([self.raw_processed_signals, new_signals], axis=0)[-self.max_len:,:]

            for i, freq in enumerate(range(60, int(self.sample_frequency/2), 60)):
                b, a = scipy.signal.iirnotch(freq, 10, self.sample_frequency)
                new_signals, self.notch_zi[i] = scipy.signal.lfilter(b, a, new_signals, axis=0, zi=self.notch_zi[i])
                new_signals, self.notch_zi_2[i] = scipy.signal.lfilter(b, a, new_signals, axis=0, zi=self.notch_zi_2[i])

            b, a = scipy.signal.butter(3, 10, 'highpass', fs=self.sample_frequency)
            new_signals, self.highpass_zi = scipy.signal.lfilter(b, a, new_signals, axis=0, zi=self.highpass_zi)

            self.processed_signals = np.concatenate([self.processed_signals, new_signals], axis=0)[-self.max_len:,:]

            self.new_samples = []

        return self.processed_signals, self.raw_processed_signals

    def get_t_values(self):
        self.t_values = self.t_values[-self.max_len:]
        return np.array(self.t_values)


class DataStream(object):
    def __init__(self):
        self.serial = serial.Serial('/dev/ttyACM0', baudrate=1000000, timeout=5)
        self.next_char = None

    def start(self):
        self.serial.reset_input_buffer()
        self.serial.write(b'S')
        while self.serial.read() != b'\xFA':
            time.sleep(0.1)
            self.serial.reset_input_buffer()
            self.serial.write(b'S')

    def stop(self):
        self.serial.write(b'E')

    def close(self):
        self.serial.flush()
        self.serial.close()

    def has_values(self):
        return self.serial.in_waiting >= 2

    def get_value(self):
        skip_count = 0
        while self.serial.read() != b's':
            skip_count += 1
        if skip_count > 0:
            print(f'skipped {skip_count} bytes')
        in_bytes = self.serial.read(4+n_channels*3)
        t = (in_bytes[0]<<24)+(in_bytes[1]<<16)+(in_bytes[2]<<8)+in_bytes[3]

        values = []
        for i in range(n_channels):
            v = (in_bytes[3*i+4]<<16)+(in_bytes[3*i+5]<<8)+in_bytes[3*i+6]

            if v >= 0x800000:
                v -= 0x1000000

            v = v * 1.2 * 1000000 / 2**23 / 16 # note: will need to adjust with gain change
            values.append(v)

        return values, t

class Display(object):
    def __init__(self, data_buffer):
        t = 5000

        plt.ion()
        fig, axis = plt.subplots(1)
        axis.axis((0,t,-1000,1000*n_channels))
        lines = axis.plot(np.zeros((t,n_channels)))
        for l,c in zip(lines, ['grey', 'mediumpurple', 'blue', 'green', 'yellow', 'orange', 'red', 'sienna']):
            l.set_color(c)
        text = axis.text(50,50,'')

        fig.tight_layout(pad=0)
        plt.show()

        def update_plot(frame):
            """ This is called by matplotlib for each plot update. """
            data, raw_data = data_buffer.get_signals()

            raw_mean = np.abs(raw_data.mean(axis=0)).max()
            raw_rms = raw_data.std(axis=0).max()

            data = data - data[:,0,np.newaxis]

            for i in range(0, n_channels):
                lines[i].set_ydata(data[:,i] + 1000*i)

            rms = data.std(axis=0).max()
            text.set_text(f'RMS: {rms:.2f} offset: {raw_mean:.2f} raw RMS: {raw_rms:.2f}')
            return lines

        self.ani = FuncAnimation(fig, update_plot, interval=30)

    def update(self):
        plt.gcf().canvas.draw_idle()
        plt.gcf().canvas.start_event_loop(0.005)


def serial_thread(queue):
    stream = DataStream()
    stream.start()
    print('started')

    try:
        while True:
            v, t = stream.get_value()
            queue.put((v, t), block=False)

    finally:
        #XXX doesn't gracefully stop on ctrl-c
        print('stopping')
        stream.stop()
        stream.close()

if __name__ == '__main__':
    q = multiprocessing.Queue()
    st = multiprocessing.Process(target=serial_thread, args=(q,))
    st.daemon = True
    st.start()

    values = []
    times = []
    display_buffer = IncrementalDataFilter(n_channels, 5000, 976.6)

    display = Display(display_buffer)
    while True:
        display.update()
        while not q.empty():
            v, t = q.get()

            values.append(v)
            times.append(t)
            display_buffer.add_sample(v, t)

            if t < 900:
                print('short time between samples')
            if t > 1030:
                print('time gap', t)

            if len(times) > 50000:
                # TODO: do this better
                times.clear()
                values.clear()


