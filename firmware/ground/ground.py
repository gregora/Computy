import pygame
import pygame.gfxdraw
from pygame import mixer
import serial
import struct
import math
import copy
import numpy as np
import pandas as pd
import time
from datetime import datetime
from threading import Thread

port = '/dev/ttyUSB0'  # Replace with your serial port
baudrate = 57600

packet_start = b'ab'

hide_location = False
recording = False
last_packet_time = time.time()
radio_connected = False

mixer.init()
connect_sound = mixer.Sound("sounds/connected.mp3")
disconnect_sound = mixer.Sound("sounds/disconnected.mp3")

columns=["Time", "Yaw", "Pitch", "Roll", "q1", "q2", "q3", "q4", "ax", "ay", "az", "Latitude", "Longitude", "Altitude", "Satellites"]

for i in range(7):
    columns.append("Channel_" + str(i))

columns.append("Mode")

saved_packets = pd.DataFrame()

now = datetime.now()
dt_string = now.strftime("%Y_%m_%d_%H:%M")
file_name = "flights/" + dt_string + ".csv"
print("Saving data to: ", file_name)

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

history = []

#

def receive_thread():
    global last_packet_time, recording, saved_packets, port, baudrate, packet_start, columns, file_name, history, p

    print(f"Port: {port}, Baudrate: {baudrate}")

    with serial.Serial(port, baudrate, timeout=1) as ser:
        while True:
            try:
                # Read the start bytes
                ch1 = ser.read(1)

                #print(ch1)

                if ch1 != packet_start[0:1]:
                    continue

                ch2 = ser.read(1)
                #print(ch2)

                if ch2 != packet_start[1:2]:
                    continue

                    # Read the rest of the packet (assuming fixed size for simplicity)
                packet_size = 60  # Adjust based on actual packet size
                
                packet_data = ser.read(packet_size)
                #print(f"{len(packet_data)}")
                #print(packet_data)

                # check that ab was not in the packet data
                if packet_start in packet_data or packet_data[-1] == packet_start[0]:
                    print("Packet start found in packet data, discarding packet")
                    continue

                #print(packet_data)
                p.read(packet_data)


                history.append(copy.deepcopy(p))

                if recording:

                    data = [p.time, yaw, pitch, roll, p.w, p.x, p.y, p.z, p.ax, p.ay, p.az, p.latitude, p.longitude, p.altitude, p.sattelites]

                    data += p.channels
                    data.append(p.mode)

                    saved_packets = pd.concat([saved_packets, pd.DataFrame([data], columns=columns)], ignore_index=True)

                last_packet_time = time.time()
            except KeyboardInterrupt:
                exit(0)
            except:
                print(f"Error reading from serial")
                continue

            #print(p)

            if len(packet_data) == packet_size:
                continue
            else:
                print("Incomplete packet received")

thread = Thread(target=receive_thread)
thread.daemon = True
thread.start()


pygame.init()
pygame.display.set_caption('Ground Station')

width = 1200
height = 800
screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)

marker_image = pygame.image.load("textures/marker.png")
location_bug_image = pygame.image.load("textures/location_bug.png")

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
    screen.blit(text, (10, 60))

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

        # flight mode
        if p.mode == 0:
            text = font_middle.render("Manual mode", True, (255, 255, 255))
        elif p.mode == 1:
            text = font_middle.render("Take-off mode", True, (17, 91, 212))
        elif p.mode == 2:
            text = font_middle.render("Fly-by-wire mode", True, (8, 138, 47))
        elif p.mode == 3:
            text = font_middle.render("Automatic mode", True, (255, 165, 0))
        elif p.mode == 255:
            text = font_middle.render("Recovery mode", True, (140, 0, 14))

        # get the width of the text
        text_rect = text.get_rect()
        screen.blit(text, (width - 115 - text_rect.width / 2, 15))




        ### Artificial horizon ###

        pitch_frac = pitch / 60

        if(abs(pitch_frac) > 1):
            pitch_frac /= abs(pitch_frac)

        pitch_frac = 0.5 + pitch_frac * 0.5

        pygame.draw.rect(screen, (30, 30, 30), (width / 2 - 205, height / 2 - 55, 410, 410))
        pygame.draw.rect(screen, (0, 125, 227), (width / 2 - 200, height / 2 - 50, 400, 400 * pitch_frac))
        pygame.draw.rect(screen, (94, 42, 0), (width / 2 - 200, height / 2 - 50 + 400 * pitch_frac, 400, 400 * (1 - pitch_frac)))


        # scale the marker
        marker = pygame.transform.scale(marker_image, (200, 16))
        # rotate the marker   
        marker = pygame.transform.rotate(marker, - roll)
        marker_rect = marker.get_rect()
        screen.blit(marker, (width / 2 - marker_rect.width/2, height / 2 + 200 - 50 - marker_rect.height/2))

        # render G-force
        g_force = (p.ax**2 + p.ay**2 + p.az**2)**0.5
        g_force = round(g_force / 10, 1)

        text = font.render(str(g_force) + " g", True, (245, 230, 66))
        screen.blit(text, (width / 2 - 190, height / 2 - 70 + 400))




        ### Minimap ###

        # render position
        pygame.draw.rect(screen, (20, 20, 20), (10, height / 2 - 30, 370, 370))

        line = []

        # get average position
        if len(history) > 1:
            avg_pos = np.array([0.0, 0.0])
            packets_with_location = 0
            for h in history[-10000::5]:
                # check if data is valid
                if h.latitude <= 0.1 and h.longitude <= 0.1:
                    continue

                if abs(h.latitude) > 180 or abs(h.longitude) > 180:
                    continue

                packets_with_location += 1

                avg_pos += np.array([h.latitude, h.longitude])
                line.append([h.latitude, h.longitude])
            if packets_with_location == 0:
                avg_pos = np.array([0.0, 0.0])
            else:
                avg_pos /= packets_with_location
            
            map_scale_lat = 40075 / 360  # full width at 1 km
            map_scale_long = 40075 / 360 * np.cos(avg_pos[0] * 3.1415 / 180) # full height at 1 km

            for l in line:
                l[0] = int(10 + 370/2 + 370 * (l[0] - avg_pos[1]) * map_scale_long)
                l[1] = int(height / 2 - 30 + 370/2 - 370 * (l[1] - avg_pos[0]) * map_scale_lat)

            if len(line) > 2:
                pygame.draw.lines(screen, (255, 255, 255), False, line, 1)

            # scale the bug
            bug = pygame.transform.smoothscale(location_bug_image, (15, 15))
            # rotate the bug   
            bug = pygame.transform.rotozoom(bug, -yaw, 1.0)
            bug_rect = bug.get_rect()

            if abs(p.latitude) < 180 and abs(p.longitude) < 180:   
                screen.blit(bug, (
                    int(10 + 370/2 + 370 * (p.longitude - avg_pos[1]) * map_scale_long - bug_rect.width/2),
                    int(height / 2 - 30 + 370/2 - 370 * (p.latitude - avg_pos[0]) * map_scale_lat) - bug_rect.height/2)
                    )



        if(time.time() - last_packet_time > 1):
            # render a red circle
            pygame.draw.circle(screen, (255, 0, 0), (width/2, 40), 20)
            
            if radio_connected:
                disconnect_sound.play()
            
            radio_connected = False
        else:
            # render a green circle
            pygame.draw.circle(screen, (0, 255, 0), (width/2, 40), 20)
            
            if not radio_connected:
                connect_sound.play()
            
            radio_connected = True

        if recording:
            pygame.draw.rect(screen, (20, 20, 20), record_button)
            # draww two vertical lines
            pygame.draw.line(screen, (255, 255, 255), (width//2 + 75, 25), (width//2 + 75, 55), 5)
            pygame.draw.line(screen, (255, 255, 255), (width//2 + 95, 25), (width//2 + 95, 55), 5)
            # add small text under the button
            font = pygame.font.Font(None, 18)
            text = font.render("Recording", True, (255, 255, 255))
            screen.blit(text, (width//2 + 55, 75))
        else:
            pygame.draw.rect(screen, (40, 40, 40), record_button)
            # draw a triangle
            pygame.draw.polygon(screen, (255, 255, 255), [(width//2 + 75, 30), (width//2 + 75, 50), (width//2 + 95, 40)])



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

    time.sleep(0.01)


if len(saved_packets) > 0:
    try:
        saved_packets.to_csv(file_name, index=False)
    except:
        saved_packets.to_csv(file_name.replace("flights/", ""), index=False)

pygame.quit()
