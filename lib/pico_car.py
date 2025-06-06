import array, time
import machine,onewire
from machine import Pin, PWM
from onewire import OneWire
import rp2
import framebuf
'''
library:
    pciocar
    ws2812b
    ultrasonic
    SSD1306
    SSD1306_I2C
    ir
    ds18b20
'''

S1 = PWM(Pin(18))
S2 = PWM(Pin(19))
S3 = PWM(Pin(20))
S4 = PWM(Pin(21))
R_B = PWM(Pin(11))
R_A = PWM(Pin(10))
L_B = PWM(Pin(13))
L_A = PWM(Pin(12))

# register definitions
SET_CONTRAST        = 0x81
SET_ENTIRE_ON       = 0xa4
SET_NORM_INV        = 0xa6
SET_DISP            = 0xae
SET_MEM_ADDR        = 0x20
SET_COL_ADDR        = 0x21
SET_PAGE_ADDR       = 0x22
SET_DISP_START_LINE = 0x40
SET_SEG_REMAP       = 0xa0
SET_MUX_RATIO       = 0xa8
SET_COM_OUT_DIR     = 0xc0
SET_DISP_OFFSET     = 0xd3
SET_COM_PIN_CFG     = 0xda
SET_DISP_CLK_DIV    = 0xd5
SET_PRECHARGE       = 0xd9
SET_VCOM_DESEL      = 0xdb
SET_CHARGE_PUMP     = 0x8d
    
@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)
def ws2812():
    T1 = 2
    T2 = 5
    T3 = 3
    label("bitloop")
    out(x, 1).side(1) [T3 - 1]
    jmp(not_x, "do_zero").side(0) [T1 - 1]
    jmp("bitloop").side(1) [T2 - 1]
    label("do_zero")
    nop().side(0) [T2 - 1]
    wrap()

class Pico_Car:
    def __init__(self):
        """
        Initialise the motors and servos of the car.

        This function is called automatically when an instance of the class is created.

        It sets the frequency of all the motors and servos to 100 Hz.
        """
        self._motors = [R_B, R_A, L_B, L_A]
        self._servos = [S1, S2, S3, S4]
        for motor in self._motors + self._servos:
            motor.freq(100)
        
    def Car_Run(self, speed1, speed2):
        """
        Drive the car at given speeds for both motors.

        This function is an alias for _set_motor_speeds.

        :param speed1: Speed for the left motor.
        :param speed2: Speed for the right motor.
        """
        self._set_motor_speeds(speed1, speed2)
        
    def Car_Stop(self):
        """
        Stop the car.

        This function is a shortcut for calling Car_Run with both speeds set to 0.
        """
        self._set_motor_speeds(0, 0)
        
    def Car_Back(self, speed1, speed2):
        """
        Drive the car backwards at given speeds for both motors.

        This function is a shortcut for calling Car_Run with both speeds set to -speed1 and -speed2.
        """
        self._set_motor_speeds(-speed1, -speed2)
        
    def Car_Left(self, speed1, speed2):
        """
        Drive the car to the left at given speeds for both motors.

        This function is a shortcut for calling Car_Run with both speeds set to speed1 and -speed2.
        """
        self._set_motor_speeds(speed1, -speed2)
        
    def Car_Right(self, speed1, speed2):
        """
        Drive the car to the right at given speeds for both motors.

        This function is a shortcut for calling Car_Run with both speeds set to -speed1 and speed2.
        """
        self._set_motor_speeds(-speed1, speed2)
        
    def _set_motor_speeds(self, speed1, speed2):
        """
        Set the speed of both motors to given values.

        :param speed1: Speed for the left motor.
        :param speed2: Speed for the right motor.
        """
        speed1 = int(speed1 * 257)
        speed2 = int(speed2 * 257)
        for motor in self._motors[::2]:
            motor.duty_u16(speed1)
        for motor in self._motors[1::2]:
            motor.duty_u16(speed2)
            
    def servo(self, num, angle, degrees=180):
        """
        Set the angle of a servo motor.

        :param num: Number of the servo motor to set.
        :param angle: Angle to set the servo motor to.
        :param degrees: Number of degrees the servo motor can move. Defaults to 180.
        """
        angle = int(angle * (360 / degrees) + 3535)
        self._servos[num-1].duty_u16(angle)
            

#delay here is the reset time. You need a pause to reset the LED strip back to the initial LED
#however, if you have quite a bit of processing to do before the next time you update the strip
#you could put in delay=0 (or a lower delay)
class ws2812b:
    """
    Controls a ws2812b LED strip.
    """

    def __init__(self, num_leds, state_machine, delay=0.001):
        """
        Initialize the LED strip.

        :param num_leds: Number of LEDs in the strip.
        :param state_machine: State machine to use for controlling the strip.
        :param delay: Delay to wait after updating the strip.
        """
        self.pixels = array.array("I", [0] * num_leds)
        self.sm = rp2.StateMachine(state_machine, ws2812, freq=8000000, sideset_base=Pin(6))
        self.sm.active(1)
        self.num_leds = num_leds
        self.delay = delay
        self.brightnessvalue = 255

    def brightness(self, brightness=None):
        """
        Set or get the brightness of the LED strip.

        :param brightness: Brightness level to set (between 1 and 255). If not set, returns the current brightness level.
        :return: The current brightness level.
        """
        if brightness is None:
            return self.brightnessvalue
        if brightness < 1:
            brightness = 1
        if brightness > 255:
            brightness = 255
        self.brightnessvalue = brightness

    def set_pixel_line_gradient(self, pixel1, pixel2, left_red, left_green, left_blue, right_red, right_green, right_blue):
        """
        Set a gradient of colors between two pixels in the strip.

        :param pixel1: Pixel number to start the gradient.
        :param pixel2: Pixel number to end the gradient.
        :param left_red: Red value of the start of the gradient.
        :param left_green: Green value of the start of the gradient.
        :param left_blue: Blue value of the start of the gradient.
        :param right_red: Red value of the end of the gradient.
        :param right_green: Green value of the end of the gradient.
        :param right_blue: Blue value of the end of the gradient.
        """
        if pixel2 - pixel1 == 0:
            return

        right_pixel = max(pixel1, pixel2)
        left_pixel = min(pixel1, pixel2)

        for i in range(right_pixel - left_pixel + 1):
            fraction = i / (right_pixel - left_pixel)
            red = round((right_red - left_red) * fraction + left_red)
            green = round((right_green - left_green) * fraction + left_green)
            blue = round((right_blue - left_blue) * fraction + left_blue)

            self.set_pixel(left_pixel + i, red, green, blue)

    def set_pixel_line(self, pixel1, pixel2, red, green, blue):
        """
        Set a line of pixels in the strip to the same color.

        :param pixel1: Pixel number to start the line.
        :param pixel2: Pixel number to end the line.
        :param red: Red value of the color.
        :param green: Green value of the color.
        :param blue: Blue value of the color.
        """
        for i in range(pixel1, pixel2+1):
            self.set_pixel(i, red, green, blue)

    def set_pixel(self, pixel_num, red, green, blue):
        """
        Set a single pixel in the strip to a color.

        :param pixel_num: Pixel number to set.
        :param red: Red value of the color.
        :param green: Green value of the color.
        :param blue: Blue value of the color.
        """
        blue = round(blue * (self.brightness() / 255))
        red = round(red * (self.brightness() / 255))
        green = round(green * (self.brightness() / 255))

        self.pixels[pixel_num] = blue | red << 8 | green << 16

    def rotate_left(self, num_of_pixels=None):
        """
        Rotate the pixel colors to the left.

        :param num_of_pixels: Number of pixels to rotate. If not set, rotates one pixel.
        """
        if num_of_pixels is None:
            num_of_pixels = 1
        self.pixels = self.pixels[num_of_pixels:] + self.pixels[:num_of_pixels]

    def rotate_right(self, num_of_pixels=None):
        """
        Rotate the pixel colors to the right.

        :param num_of_pixels: Number of pixels to rotate. If not set, rotates one pixel.
        """
        if num_of_pixels is None:
            num_of_pixels = 1
        num_of_pixels = -1 * num_of_pixels
        self.pixels = self.pixels[num_of_pixels:] + self.pixels[:num_of_pixels]

    def show(self):
        """
        Update the LED strip with the current pixel colors.
        """
        for i in range(self.num_leds):
            self.sm.put(self.pixels[i], 8)
        time.sleep(self.delay)

    def fill(self, red, green, blue):
        """
        Fill the entire strip with a single color.

        :param red: Red value of the color.
        :param green: Green value of the color.
        :param blue: Blue value of the color.
        """
        for i in range(self.num_leds):
            self.set_pixel(i, red, green, blue)
        time.sleep(self.delay)


class Ultrasonic:
    def __init__(self):
        self.trig = Pin(0, Pin.OUT)
        self.echo = Pin(1, Pin.IN)

    def _send_pulse(self):
        self.trig.value(0)
        time.sleep_us(2)
        self.trig.value(1)
        time.sleep_us(10)
        self.trig.value(0)

    def _get_pulse_duration(self):
        while not self.echo.value():
            pass
        start = time.ticks_us()

        while self.echo.value():
            pass
        end = time.ticks_us()

        return time.ticks_diff(end, start)

    def distance(self):
        self._send_pulse()
        duration = self._get_pulse_duration()
        return (duration * 0.0343) / 2

    def distance_accurate(self):
        measurements = (self.distance() for _ in range(5))
        average_distance = sum(measurements) / 5
        return average_distance if 0 < average_distance <= 500 else 999


class SSD1306:
    def __init__(self, width, height, external_vcc):
        self.width = width
        self.height = height
        self.external_vcc = external_vcc
        self.pages = self.height
        # Note the subclass must initialize self.framebuf to a framebuffer.
        # This is necessary because the underlying data buffer is different
        # between I2C and SPI implementations (I2C needs an extra byte).
        self.poweron()
        self.init_display()

    def init_display(self):
        for cmd in (
            SET_DISP | 0x00, # off
            # address setting
            SET_MEM_ADDR, 0x00, # horizontal
            # resolution and layout
            SET_DISP_START_LINE | 0x00,
            SET_SEG_REMAP | 0x01, # column addr 127 mapped to SEG0
            SET_MUX_RATIO, self.height - 1,
            SET_COM_OUT_DIR | 0x08, # scan from COM[N] to COM0
            SET_DISP_OFFSET, 0x00,
            SET_COM_PIN_CFG, 0x02 if self.height == 32 else 0x12,
            # timing and driving scheme
            SET_DISP_CLK_DIV, 0x80,
            SET_PRECHARGE, 0x22 if self.external_vcc else 0xf1,
            SET_VCOM_DESEL, 0x30, # 0.83*Vcc
            # display
            SET_CONTRAST, 0xff, # maximum
            SET_ENTIRE_ON, # output follows RAM contents
            SET_NORM_INV, # not inverted
            # charge pump
            SET_CHARGE_PUMP, 0x10 if self.external_vcc else 0x14,
            SET_DISP | 0x01): # on
            self.write_cmd(cmd)
        self.fill(0)
        self.show()

    def poweroff(self):
        self.write_cmd(SET_DISP | 0x00)

    def contrast(self, contrast):
        self.write_cmd(SET_CONTRAST)
        self.write_cmd(contrast)

    def invert(self, invert):
        self.write_cmd(SET_NORM_INV | (invert & 1))

    def show(self):
        x0 = 0
        x1 = self.width - 1
        if self.width == 64:
            # displays with width of 64 pixels are shifted by 32
            x0 += 32
            x1 += 32
        self.write_cmd(SET_COL_ADDR)
        self.write_cmd(x0)
        self.write_cmd(x1)
        self.write_cmd(SET_PAGE_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.pages - 1)
        self.write_framebuf()

    def fill(self, col):
        self.framebuf.fill(col)

    def pixel(self, x, y, col):
        self.framebuf.pixel(x, y, col)

    def scroll(self, dx, dy):
        self.framebuf.scroll(dx, dy)

    def text(self, string, x, y, col=1):
        self.framebuf.text(string, x, y, col)


class SSD1306_I2C(SSD1306):
    def __init__(self, width, height, i2c, addr=0x3c, external_vcc=False):
        self.i2c = i2c
        self.addr = addr
        self.temp = bytearray(2)
        # Add an extra byte to the data buffer to hold an I2C data/command byte
        # to use hardware-compatible I2C transactions.  A memoryview of the
        # buffer is used to mask this byte from the framebuffer operations
        # (without a major memory hit as memoryview doesn't copy to a separate
        # buffer).
        self.buffer = bytearray(((height // 8) * width) + 1)
        self.buffer[0] = 0x40  # Set first byte of data buffer to Co=0, D/C=1
        self.framebuf = framebuf.FrameBuffer1(memoryview(self.buffer)[1:], width, height)
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        self.temp[0] = 0x80 # Co=1, D/C#=0
        self.temp[1] = cmd
        self.i2c.writeto(self.addr, self.temp)

    def write_framebuf(self):
        # Blast out the frame buffer using a single I2C transaction to support
        # hardware I2C interfaces.
        self.i2c.writeto(self.addr, self.buffer)

    def poweron(self):
        pass


class ir():
    def __init__(self):
        self.Pin = Pin(7)
        self.Pin.value(1)
        self.ir_repeat_cnt = 0
        self.irdata = 0xfe
            
    def Getir(self):
        if self.Pin.value() == 0:
            
            self.ir_repeat_cnt = 0
            count = 0
            while self.Pin.value() == 0:
                count += 1
                time.sleep(0.00003)

            count = 0
            while self.Pin.value() == 1 and count < 160:
                count += 1
                time.sleep(0.00003)
            #print("")
            idx = 0
            cnt = 0
            data = [0,0,0,0]
            for i in range(0,32):
                count = 0
                while self.Pin.value() == 0 and count < 30:
                    count += 1
                    time.sleep(0.00003)

                count = 0
                while self.Pin.value() == 1 and count < 80:
                    count += 1
                    time.sleep(0.00003)

                if count > 35:
                    data[idx] |= 1<<cnt
                if cnt == 7:
                    cnt = 0
                    idx += 1
                else:
                    cnt += 1
            
            if data[0]+data[1] == 0xFF and data[2]+data[3] == 0xFF:
                self.irdata = data[2]
        else:
            if self.ir_repeat_cnt > 110: 
                self.ir_repeat_cnt = 0
                self.irdata = 0xfe
            else:
                time.sleep(0.001)
                self.ir_repeat_cnt += 1
        if self.irdata != None:
            if self.irdata != 254:
                return self.irdata

class ds:
    def __init__(self, unit='F', resolution=12):
        self.pin=7
        self.no_addr=0
        self.addr=self.getaddr()
        self.unit=unit
        self.res=resolution

    def getaddr(self):
        ow=OneWire(Pin(self.pin))
        a=ow.scan()
        for i in a:
            self.no_addr+=1
        return a
        
    def read(self):
        if self.no_addr==0:
            print ("no sensors detected")
        if self.no_addr>=1:
            temp_array=[]
            #print ('number of sensors: ',self.no_addr)
            for i in range(1,self.no_addr+1):
                temp_array.append(self._request(self.addr[i-1]))
                return temp_array       
        
    def _request(self, addr):
        self._res(addr)
        ow=OneWire(Pin(self.pin))
        ow.reset()
        ow.select_rom(addr)
        ow.writebyte(0x44) #command to take reading
        if self.res==12: #the resolution determines the amount of time needed
            time.sleep_ms(1000)
        if self.res==11:
            time.sleep_ms(400)
        if self.res==10:
            time.sleep_ms(200)
        if self.res==9:
            time.sleep_ms(100)
        ow.reset() #reset required for data
        ow.select_rom(addr)
        ow.writebyte(0xBE) #command to send temperature data
        #all nine bytes must be read
        LSB=ow.readbyte() #least significant byte
        MSB=ow.readbyte() #most significant byte
        ow.readbyte()
        ow.readbyte()
        ow.readbyte() #this is the configuration byte for resolution
        ow.readbyte()
        ow.readbyte()
        ow.readbyte()
        ow.readbyte()
        ow.reset() #reset at end of data transmission
        #convert response to binary, format the binary string, and perform math
        d_LSB=float(0)
        d_MSB=float(0)
        count=0
        b=bin(LSB)
        b2=bin(MSB)
        b3=""
        l=10-len(b2)
        for i in range(l):
            if len(b2)<10:
                b3+="0"
        b2=b3+b2
        b4=""
        l=10-len(b)
        for i in range(l):
            if len(b)<10:
                b4+="0"
        b5=b4+b
        for i in b5:
            if count == 2:
                if i=='1':
                    d_LSB+=2**3
            if count == 3:
                if i=='1':
                    d_LSB+=2**2
            if count == 4:
                if i=='1':
                    d_LSB+=2**1
            if count == 5:
                if i=='1':
                    d_LSB+=2**0
            if count == 6:
                if i=='1':
                    d_LSB+=2**-1
            if count == 7:
                if i=='1':
                    d_LSB+=2**-2
            if count == 8:
                if i=='1':
                    d_LSB+=2**-3
            if count == 9:
                if i=='1':
                    d_LSB+=2**-4
            count+=1
        count=0
        sign=1
        for i in b2:
            if count == 6:
                if i=='1':
                    sign=-1
            if count == 7:
                if i=='1':
                    d_MSB+=2**6
            if count == 8:
                if i=='1':
                    d_MSB+=2**5
            if count == 9:
                if i=='1':
                    d_MSB+=2**4
            count+=1
        temp=(d_LSB+d_MSB)*sign
        '''
        if self.unit=='c'or self.unit=='C':
            print("TEMP is: "+str(temp)+" degrees C")
        if self.unit=='f'or self.unit=='F':
            temp=(temp*9/5)+32
            print("TEMP F is: "+str(temp))
        '''
        return temp

    def _res(self,addr):
        ow=OneWire(Pin(self.pin))
        ow.reset()
        ow.select_rom(addr)
        ow.writebyte(0x4E)
        if self.res==12:
            ow.writebyte(0x7F)
            ow.writebyte(0x7F)
            ow.writebyte(0x7F)
            #print ("12 bit mode")
        if self.res==11:
            ow.writebyte(0x5F)
            ow.writebyte(0x5F)
            ow.writebyte(0x5F)
            #print ("11 bit mode")
        if self.res==10:
            ow.writebyte(0x3F)
            ow.writebyte(0x3F)
            ow.writebyte(0x3F)
            #print ("10 bit mode")
        if self.res==9:
            ow.writebyte(0x1F)
            ow.writebyte(0x1F)
            ow.writebyte(0x1F)
            #print ("9 bit mode")
        ow.reset()  


