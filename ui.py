import serial
import tkinter as tk
from tkinter import scrolledtext, Entry, Label, Button
import struct
from enum import Enum


BAUD_RATE = 115200
ser = serial.Serial("COM3", BAUD_RATE)


# def but_get_temp(current_temp_text):
#     global last_read_request_reg_count
#     last_read_request_reg_count = 1
#     r_req = R_Req(
#         header=Header(dev_id=0x1, cmd=CMD_R_AO),
#         first_reg_idx=GET_INTERNAL_TEMP_AI1,
#         reg_count=last_read_request_reg_count,
#     )
#     ser.write(r_req.serialize())
#     req_size = get_read_response_size(r_req)
#     data = bytearray()
#     while len(data) < req_size:
#         if ser.in_waiting == 0:
#             continue
#         data.extend(ser.read_all())
#         fil = list(filter(lambda line: not is_dbg_log(line), data.split(b"\n")))
#         if len(fil) == 0:
#             data.clear()
#         else:
#             data = fil[0]

#     print(data)

#     # current_temp_text.config(text="sa")


def crc32b(packet_buf: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in packet_buf:
        crc = crc ^ byte
        for _ in range(8):
            mask = -(crc & 1)
            crc = (crc >> 1) ^ (0xEDB88320 & mask)
    return ~crc & 0xFFFFFFFF


def print_buf(arr):
    s = ""
    for i, x in enumerate(arr):
        if i % 4 == 0:
            s += " | " + "0x{:02x}".format(x)
        else:
            s += " 0x{:02x}".format(x)
    print(s)


SET_PID_COEFFS_AO1_3 = 1
SET_POINT_REG_AO0 = 0
GET_THERMISTOR_TEMP_AI0 = 0
GET_INTERNAL_TEMP_AI1 = 1
GET_STATUS_AI2 = 2


CMD_R_AO = 0
CMD_R_AI = 1
CMD_W_AI = 2
CMD_W_AO = 3


class Header:
    def __init__(self, dev_id: int, cmd: int):
        self.dev_id = dev_id
        self.cmd = cmd

    def serialize(self) -> bytearray:
        ba = bytearray()
        ba.extend(self.dev_id.to_bytes(1, byteorder="little"))
        ba.extend(self.cmd.to_bytes(1, byteorder="little"))
        return ba


class R_Req:
    def __init__(self, header: Header, first_reg_idx: int, reg_count: int):
        self.header = header
        self.first_reg_idx = first_reg_idx
        self.reg_count = reg_count

    def serialize(self) -> bytearray:
        ba = bytearray()
        ba.extend(self.header.serialize())
        ba.extend(self.first_reg_idx.to_bytes(2, byteorder="little"))
        ba.extend(self.reg_count.to_bytes(2, byteorder="little"))
        ba.extend((0xCCCF).to_bytes(2, byteorder="little"))  # TODO: calculate crc
        return ba

    def get_response_size(self):
        return 1 + 1 + self.reg_count * 4 + 1 + 2

    def block_until_resp(self):
        req_size = self.get_response_size()
        while ser.in_waiting < req_size:
            pass

    def check_resp_and_return_data(self, data: bytearray):
        assert data[0] == self.header.dev_id
        assert data[1] == self.header.cmd
        _error = int.from_bytes(
            data[len(data) - 3 : len(data) - 2], byteorder="little", signed=False
        )  # TODO: check error
        _crc = int.from_bytes(
            data[len(data) - 2 : len(data)], byteorder="little", signed=False
        )  # TODO: calc. and check crc

        return data[2 : len(data) - 3]


class W_Req:
    def __init__(
        self,
        header: Header,
        first_reg_idx: int,
        reg_count: int,
        reg_vals: list[int],
    ):
        self.header = header
        self.first_reg_idx = first_reg_idx
        self.reg_count = reg_count
        self.reg_vals = reg_vals
        self.req_size = None

    def serialize(self) -> bytearray:
        ba = bytearray()
        ba.extend(self.header.serialize())
        ba.extend(self.first_reg_idx.to_bytes(2, byteorder="little"))
        ba.extend(self.reg_count.to_bytes(2, byteorder="little"))
        for reg_val in self.reg_vals:
            if isinstance(reg_val, int):
                ba.extend(reg_val.to_bytes(4, byteorder="little", signed=False))
            elif isinstance(reg_val, float):
                ba.extend(bytearray(struct.pack("f", reg_val)))

        ba.extend((0xCCCC).to_bytes(2, byteorder="little"))  # calculate crc
        self.req_size = len(ba)
        print_buf(ba)
        return ba

    def get_response_size(self):
        assert self.req_size is not None
        return self.req_size + 1  # + error

    def block_until_resp(self):
        req_size = self.get_response_size()
        while ser.in_waiting < req_size:
            pass

    def check_resp_and_return_data(self, data: bytearray):
        assert data[0] == self.header.dev_id
        assert data[1] == self.header.cmd

        # TODO: must be the same req + error + crc
        #

        _error = int.from_bytes(
            data[len(data) - 3 : len(data) - 2], byteorder="little", signed=False
        )  # TODO: check error
        _crc = int.from_bytes(
            data[len(data) - 2 : len(data)], byteorder="little", signed=False
        )  # TODO: calc. and check crc

        return data[2 : len(data) - 3]


def but_set_temp(temp):
    w_req = W_Req(
        header=Header(dev_id=0x1, cmd=CMD_W_AO),
        first_reg_idx=SET_POINT_REG_AO0,
        reg_count=1,
        reg_vals=[int(temp)],
    )
    ser.write(w_req.serialize())
    w_req.block_until_resp()
    data = ser.read_all()
    resp = w_req.check_resp_and_return_data(data)
    print(resp)


def is_dbg_log(data: bytearray):
    try:
        return data.decode("utf-8").startswith("DBG")
    except UnicodeDecodeError:
        pass


def but_get_temp(current_temp_text):
    r_req = R_Req(
        header=Header(dev_id=0x1, cmd=CMD_R_AI),
        first_reg_idx=GET_THERMISTOR_TEMP_AI0,
        reg_count=1,
    )
    ser.write(r_req.serialize())
    r_req.block_until_resp()
    data = ser.read_all()
    packet = r_req.check_resp_and_return_data(data)
    reg1 = int.from_bytes(packet, byteorder="little", signed=True)
    current_temp_text.config(text=reg1)


def set_pid_coeffs_cmd(p, i, d):
    if not p or not i or not d:
        return
    w_req = W_Req(
        header=Header(dev_id=0x1, cmd=CMD_W_AO),
        first_reg_idx=SET_PID_COEFFS_AO1_3,
        reg_count=3,
        reg_vals=[float(p), float(i), float(d)],
    )
    ser.write(w_req.serialize())
    w_req.block_until_resp()
    data = ser.read_all()
    resp = w_req.check_resp_and_return_data(data)
    print(resp)


def set_clock_cmd(clock: str):
    try:
        [h, m, s] = clock.split(":")
    except Exception:
        print("wrong clock format")
        return
    raise Exception("not implementd yet")


STATUS_STANDBY = 0
STATUS_SET_TEMP = 1
STATUS_TEMP_STABLE = 2
STATUS_FAULT = 3
STATUS_STANDBY_IT = 4


class App:
    def __init__(self, root):
        self.root = root

        self.root.title("Serial")

        Label(root, text="Target Temp:").grid(
            row=0, column=0, padx=10, pady=10, sticky="n"
        )

        self.status_label = Label(root, text="Status:").grid(
            row=0, column=0, padx=10, pady=10
        )

        self.status_label = Label(root)
        self.status_label.grid(row=0, column=1, padx=10, pady=10)

        self.current_temp_text = Label(root, width=10)
        self.current_temp_text.config(text="-")
        self.current_temp_text.grid(row=1, column=1, padx=0, pady=10, sticky="n")

        self.text_widget = scrolledtext.ScrolledText(
            root, width=50, height=10, state=tk.DISABLED
        )
        self.text_widget.grid(row=0, column=2, padx=10, pady=10)

        self.set_temp_text = Entry(root, width=10)
        self.set_temp_text.grid(row=0, column=1, padx=0, pady=10, sticky="n")
        self.set_temp_text.insert(0, "99")

        self.set_temp_but = Button(
            root,
            text="Set Temp",
            command=lambda: but_set_temp(self.set_temp_text.get()),
        )
        self.set_temp_but.grid(row=3, column=0, padx=10, pady=10)

        self.get_temp_but = Button(
            root,
            text="Get Temp",
            command=lambda: but_get_temp(self.current_temp_text),
        )
        self.get_temp_but.grid(row=2, column=0, padx=10, pady=10)

        # pid coeffs
        Label(root, text="P:").grid(row=4, column=0, sticky="n")
        self.pid_p = Entry(root, width=10)
        self.pid_p.insert(0, "3")
        self.pid_p.grid(row=4, column=1)

        Label(root, text="I:").grid(row=5, column=0, sticky="n")
        self.pid_i = Entry(root, width=10)
        self.pid_i.insert(0, "0")
        self.pid_i.grid(row=5, column=1)

        Label(root, text="D:").grid(row=6, column=0, sticky="n")
        self.pid_d = Entry(root, width=10)
        self.pid_d.insert(0, "0")
        self.pid_d.grid(row=6, column=1)

        self.set_pid_coeffs = Button(
            root,
            text="Set PID coefficients",
            command=lambda: set_pid_coeffs_cmd(
                self.pid_p.get(), self.pid_i.get(), self.pid_d.get()
            ),
        )
        self.set_pid_coeffs.grid(row=7, column=2, padx=10, pady=10)

        # clock
        Label(root, text="Clock:").grid(row=2, column=2)
        self.clock_entry = Entry(root, width=18)
        self.clock_entry.insert(0, "00:00:00")
        self.clock_entry.grid(row=2, column=2)
        self.set_clock_but = Button(
            root,
            text="Set Clock",
            command=lambda: set_clock_cmd(self.clock_entry.get()),
        )
        self.set_clock_but.grid(row=3, column=2, padx=10, pady=10)

        # root.grid_rowconfigure(1, weight=0)
        self.root.after(200, self.poll_serial)
        self.root.after(1000, self.poll_status)

    def poll_serial(self):
        if ser.in_waiting > 4:
            data = ser.read_all()
            if is_dbg_log(data):
                self.text_widget.configure(state=tk.NORMAL)
                self.text_widget.insert(tk.END, data)
                self.text_widget.yview(tk.END)
                self.text_widget.configure(state=tk.DISABLED)

        self.root.after(200, self.poll_serial)

    def poll_status(self):
        r_req = R_Req(
            header=Header(dev_id=0x1, cmd=CMD_R_AI),
            first_reg_idx=GET_STATUS_AI2,
            reg_count=1,
        )
        ser.write(r_req.serialize())
        r_req.block_until_resp()
        data = ser.read_all()
        packet = r_req.check_resp_and_return_data(data)
        status = int.from_bytes(packet, byteorder="little", signed=True)
        if status == STATUS_STANDBY:
            status_text = "STANDBY"
        elif status == STATUS_SET_TEMP:
            status_text = "SET TEMP"
        elif status == STATUS_TEMP_STABLE:
            status_text = "TEMP STABLE"
        elif status == STATUS_FAULT:
            status_text = "FAULT"
        elif status == STATUS_STANDBY_IT:
            status_text = "STANDBY IT"
        else:
            status_text = "-"

        self.status_label.config(text=status_text)

        self.root.after(1000, self.poll_status)


if __name__ == "__main__":
    root = tk.Tk()

    app = App(root)

    root.resizable(False, False)
    root.mainloop()
