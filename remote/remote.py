import time
import pygame
import threading
import queue

from pySerialTransfer import pySerialTransfer as txfer

SERIAL_PORT = "/dev/ttyACM1"
ARDUINO_REST_TIME = 4
APPLICATION_NAME = "Drone Remote"
SCREEN_RESOLUTION = (500, 700)
senderThreadDone = threading.Event()
ABORT_BUTTON = 0
YAW_RIGHT_BUTTON = 1
YAW_LEFT_BUTTON = 2

class ControlData:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.throttle = 0.0
        self.abort = 0

controlDataQueue = queue.Queue()

class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 25)

    def tprint(self, screen, text):
        text_bitmap = self.font.render(text, True, (0, 0, 0))
        screen.blit(text_bitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10

def senderThreadTask(senderThreadDone):
    while not senderThreadDone.is_set():
        link = None
        try:
            link = txfer.SerialTransfer(SERIAL_PORT)
            link.open()
            
            while not senderThreadDone.is_set():
                send_size = 0
                controlData = controlDataQueue.get()
                if controlData is None:
                    break
                send_size = link.tx_obj(controlData.abort, send_size)
                send_size = link.tx_obj(controlData.roll, send_size)
                send_size = link.tx_obj(controlData.pitch, send_size)
                send_size = link.tx_obj(controlData.yaw, send_size)

                float_size = send_size
                send_size = link.tx_obj(controlData.throttle, send_size)
                float_size = send_size - float_size
                
                link.send(send_size)
                time.sleep(0.001)
                # print(f"{controlData.pitch}")
                controlDataQueue.task_done()

                if link.available():
                    speedA = 0.0
                    float_size = 4
                    speedA = link.rx_obj(obj_type=type(speedA), obj_byte_size=float_size, start_pos=(float_size*0))
                    speedB = link.rx_obj(obj_type=type(speedA), obj_byte_size=float_size, start_pos=(float_size*1))
                    speedC = link.rx_obj(obj_type=type(speedA), obj_byte_size=float_size, start_pos=(float_size*2))
                    speedD = link.rx_obj(obj_type=type(speedA), obj_byte_size=float_size, start_pos=(float_size*3))
                    print(f"{speedA}, {speedB}, {speedC}, {speedD}")
            
            link.close()

        except Exception as e:
            print(e)
            if link is not None:
                link.close()

def main():
    senderThread = threading.Thread(target=senderThreadTask, args=(senderThreadDone,))
    senderThread.start()

    try:
        pygame.init()

        screen = pygame.display.set_mode(SCREEN_RESOLUTION)
        pygame.display.set_caption(APPLICATION_NAME)
        clock = pygame.time.Clock()

        screenPrint = TextPrint()

        time.sleep(ARDUINO_REST_TIME)

        joysticks = {}

        done = False
        while not done:
            send_size = 0

            # Handle pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True 

                # Handle hotplugging
                if event.type == pygame.JOYDEVICEADDED:
                    # This event will be generated when the program starts for every
                    # joystick, filling up the list without needing to create them manually.
                    joy = pygame.joystick.Joystick(event.device_index)
                    joysticks[joy.get_instance_id()] = joy
                    print(f"Joystick {joy.get_instance_id()} connencted")

                if event.type == pygame.JOYDEVICEREMOVED:
                    del joysticks[event.instance_id]
                    print(f"Joystick {event.instance_id} disconnected")

            # If no joysticks found, don't bother
            if len(joysticks) > 0:
                controlData = ControlData()

                screen.fill((255, 255, 255))
                screenPrint.reset()
                
                joystick = joysticks[len(joysticks)-1]

                abortButton = int(joystick.get_button(ABORT_BUTTON))


                controlData.pitch = joystick.get_axis(0)
                screenPrint.tprint(screen, f"pitch value: {controlData.pitch:>6.3f}")

                controlData.roll = joystick.get_axis(1)
                screenPrint.tprint(screen, f"roll value: {controlData.roll:>6.3f}")

                if joystick.get_button(YAW_RIGHT_BUTTON) > 0:
                    controlData.yaw = 1.0
                elif joystick.get_button(YAW_LEFT_BUTTON) > 0:
                    controlData.yaw = -1.0
                screenPrint.tprint(screen, f"yaw value: {controlData.yaw:>6.3f}")

                controlData.throttle = -joystick.get_axis(2)
                screenPrint.tprint(screen, f"throttle value: {controlData.throttle:>6.3f}")

                controlData.abort = abortButton
                screenPrint.tprint(screen, f"Abort button pressed: {abortButton}")

                controlDataQueue.put(controlData)
            
            pygame.display.flip()
            clock.tick(30)
    
    except:
        import traceback
        traceback.print_exc()
    
    print("quiting...")
    senderThreadDone.set()
    controlDataQueue.join()
    controlDataQueue.put(None)
    senderThread.join()

if __name__ == "__main__":
    main()


