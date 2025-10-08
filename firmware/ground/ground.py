import pygame
import serial
import struct
import math

port = '/dev/ttyUSB0'  # Replace with your serial port
baudrate = 57600

packet_start = b'ab'

hide_location = False


def quat2ZYX(w, x, y, z):
    # Convert quaternion to ZYX Euler angles (yaw, pitch, roll)

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    X = math.atan2(sinr_cosp, cosr_cosp)  # roll
    # roll (X-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        Y = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        Y = math.asin(sinp)  # pitch
    # pitch (Y-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    Z = math.atan2(siny_cosp, cosy_cosp)  # yaw
    # yaw (Z-axis rotation)

    return Z, Y, X  # in radians


class Packet:
    time = 0
    w = 0.0
    x = 0.0
    y = 0.0
    z = 0.0
    ax = 0.0
    ay = 0.0
    az = 0.0

    latitude = 0.0
    longitude = 0.0
    altitude = 0

    sattelites = 0

    channels = [0] * 7

    mode = 0

    def read(self, bytes_data):

        data = struct.unpack("<Ifffffffffhh7hh", bytes_data)

        (
            self.time,
            self.w,
            self.x,
            self.y,
            self.z,
            self.ax,
            self.ay,
            self.az,
            self.latitude,
            self.longitude,
            self.altitude,
            self.sattelites,
            *self.channels,
            self.mode
        ) = data

    def __str__(self):
        string = "" \
        "Time: {}\n" \
        "Orientation: ({}, {}, {}, {})\n" \
        "Acceleration: ({}, {}, {})\n" \
        "GPS: (lat: {}, lon: {}, alt: {}, sats: {})\n" \
        "Channels: {}\n" \
        "Mode: {}\n".format(
            self.time,
            self.w, self.x, self.y, self.z,
            self.ax, self.ay, self.az,
            self.latitude, self.longitude, self.altitude, self.sattelites,
            self.channels,
            self.mode
        )
        return string

p = Packet()

with serial.Serial(port, baudrate, timeout=1) as ser:

    pygame.init()
    pygame.display.set_caption('Ground Station')

    width = 1200
    height = 800
    screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)

    marker_image = pygame.image.load("marker.png")

    last_packet_time = 0

    running = True


    while running:

        record_button = pygame.Rect(width//2 + 60, 15, 50, 50)
        screen.fill((50, 50, 50))


        pygame.draw.rect(screen, (20, 20, 20), (0, 0, width/2 + 50, 275))

        font = pygame.font.Font(None, 20)
        font_middle = pygame.font.Font(None, 30)
        font_big = pygame.font.Font(None, 40)

        yaw, pitch, roll = quat2ZYX(p.w, p.x, p.y, p.z)
        yaw = math.degrees(yaw)
        pitch = math.degrees(pitch)
        roll = math.degrees(roll)

        # left side
        text = font.render("Yaw: " + str(round(yaw, 2)), True, (255, 255, 255))
        screen.blit(text, (10, 10))

        text = font.render("Pitch: " + str(round(pitch, 2)), True, (255, 255, 255))
        screen.blit(text, (10, 35))

        text = font.render("Roll: " + str(round(roll, 2)), True, (255, 255, 255))
        screen.blit(text, (10, 85))

        text = font.render("ax: " + str(round(p.ax, 2)), True, (255, 255, 255))
        screen.blit(text, (10, 110))

        text = font.render("ay: " + str(round(p.ay, 2)), True, (255, 255, 255))
        screen.blit(text, (10, 135))

        text = font.render("az: " + str(round(p.az, 2)), True, (255, 255, 255))
        screen.blit(text, (10, 160))


        text = font.render("Channels: " + str(p.channels), True, (255, 255, 255))
        screen.blit(text, (10, 210))



        text = font.render("Time: " + str(p.time), True, (255, 255, 255))
        screen.blit(text, (200, 10))

        text = font.render("Mode: " + str(p.mode), True, (255, 255, 255))
        screen.blit(text, (200, 35))

        if not hide_location:

            text = font.render("Latitude: " + str(p.latitude), True, (255, 255, 255))
            screen.blit(text, (200, 85))

            text = font.render("Longitude: " + str(p.longitude), True, (255, 255, 255))
            screen.blit(text, (200, 110))

        text = font.render("Altitude: " + str(p.altitude), True, (255, 255, 255))
        screen.blit(text, (200, 135))

        text = font.render("Satellites: " + str(p.sattelites), True, (255, 255, 255))
        screen.blit(text, (200, 160))



        text = font_big.render("Elapsed: " + str(round(p.time / 1000, 1)) + " s", True, (255, 255, 255))
        screen.blit(text, (10, 285))

        # visualize the channels
        for i, c in enumerate(p.channels):
            pygame.draw.rect(screen, (255, 255, 255), (width + (i - 14) * 15 - 10, 265 - c//10, 10, c // 10))
            text = font.render(str(i), True, (255, 255, 255))
            screen.blit(text, (width + (i - 14) * 15 - 10, 270))

        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                if record_button.collidepoint(event.pos):
                    recording = not recording
            # check if space is pressed
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    recording = not recording

        width, height = screen.get_size()

        # Read the start bytes

        if ser.read(1) != packet_start[0:1]:
            continue

        if ser.read(1) != packet_start[1:2]:
            continue

        # Read the rest of the packet (assuming fixed size for simplicity)
        packet_size = 60  # Adjust based on actual packet size
        try:
            packet_data = ser.read(packet_size)
            #print(packet_data)
            p.read(packet_data)
        except KeyboardInterrupt:
            exit(0)
        except:
            print(f"Error reading from serial")
            continue




        print(p)

        if len(packet_data) == packet_size:
            continue
        else:
            print("Incomplete packet received")


    pygame.quit()
