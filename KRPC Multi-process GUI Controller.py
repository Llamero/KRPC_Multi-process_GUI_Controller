#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      Ben
#
# Created:     09/07/2020
# Copyright:   (c) Ben 2020
# Licence:     <your licence>
#-------------------------------------------------------------------------------
import krpc
from multiprocessing import Process, Value, Array, Manager, Pipe, connection
import time
import datetime
import operator #Allow strings to be converted to class attributes
import traceback
import math
from tkinter import *
import tkinter.font as tkFont
from tkinter.ttk import *
import sys
from collections import OrderedDict

def server_stream(stream_flight_dict, stream_vessel_dict, stream_orbit_dict, stream_body_dict, stream_ksc_dict, kill_flag):
    #try:
        conn = krpc.connect(name='Server stream')
        vessel = conn.space_center.active_vessel
        body = vessel.orbit.body
        srf_frame = vessel.surface_reference_frame
        obt_frame = vessel.orbit.body.non_rotating_reference_frame
        flight_conn_dict = dict()
        ksc_conn_dict = dict()
        vessel_conn_dict = dict()
        orbit_conn_dict = dict()
        no_stream = ["biome", "thrust", "available_thrust", "max_thrust", "specific_impulse", "mass"]
        cur_body = None #Tracking whether the body has changed
        prev_body = None

        for key, value in stream_flight_dict.items():
            flight_conn_dict[key] = conn.add_stream(getattr, vessel.flight(srf_frame), key) #Setup flight attribute streams
        for key, value in stream_ksc_dict.items():
            ksc_conn_dict[key] = conn.add_stream(getattr, conn.space_center, key) #Setup flight attribute streams



        for key, value in stream_vessel_dict.items():
            if(key not in no_stream): #remove atttributes that don't have a refrence frame
                vessel_conn_dict[key] = conn.add_stream(operator.attrgetter(key)(vessel), srf_frame) #Setup vessel class streams

        orbit_frame = False

        while kill_flag.value == 0:
            #n1=datetime.datetime.now()
            if(not orbit_frame and stream_flight_dict["mean_altitude"] is not None and stream_body_dict["atmosphere_depth"] is not None): #Switch to orbit frame if above atmosphere
                if (stream_flight_dict["mean_altitude"] > stream_body_dict["atmosphere_depth"]):
                    srf_frame = vessel.orbit.body.reference_frame
                    for key, value in stream_flight_dict.items():
                        flight_conn_dict[key] = conn.add_stream(getattr, vessel.flight(srf_frame), key) #Setup flight attribute streams
                    for key, value in stream_vessel_dict.items(): #Switch to surface frame if below atmosphere
                        if(key not in no_stream): #remove atttributes that don't have a refrence frame
                            vessel_conn_dict[key] = conn.add_stream(operator.attrgetter(key)(vessel), srf_frame) #Setup vessel class streams
                    orbit_frame = True

            elif(orbit_frame and stream_flight_dict["mean_altitude"] is not None and stream_body_dict["atmosphere_depth"] is not None):
                if (stream_flight_dict["mean_altitude"] < stream_body_dict["atmosphere_depth"]):
                    srf_frame = vessel.surface_reference_frame
                    for key, value in stream_flight_dict.items():
                        flight_conn_dict[key] = conn.add_stream(getattr, vessel.flight(srf_frame), key) #Setup flight attribute streams
                    for key, value in stream_vessel_dict.items():
                        if(key not in no_stream): #remove atttributes that don't have a refrence frame
                            vessel_conn_dict[key] = conn.add_stream(operator.attrgetter(key)(vessel), srf_frame) #Setup vessel class streams
                    orbit_frame = False

            for key, value in stream_flight_dict.items():
                stream_flight_dict[key] = flight_conn_dict[key]()
            for key, value in stream_ksc_dict.items():
                stream_ksc_dict[key] = ksc_conn_dict[key]()
            for key, value in stream_vessel_dict.items():
                if(key in no_stream): #Directly get attributes that don't have a reference frame
                    stream_vessel_dict[key] = operator.attrgetter(key)(vessel)
                else:
                    stream_vessel_dict[key] = vessel_conn_dict[key]()
            for key, value in stream_orbit_dict.items():
                stream_orbit_dict[key] = operator.attrgetter("orbit." + str(key))(vessel)
            cur_body = body.name #Update the celestial body info if it has changed
            if(cur_body != prev_body):
                prev_body = cur_body
                for key, value in stream_body_dict.items():
                    stream_body_dict[key] = operator.attrgetter(key)(body)

            #n2=datetime.datetime.now()
            #print("Seconds to complete: " + str((n2-n1).microseconds/1e6))
            time.sleep(0.1)
    #except:
        print("Stream process error")

def auto_pilot(flight_dict, ksc_dict, gui_pipe, kill_flag):
##def auto_pilot(flight_dict, ksc_dict):
    conn = krpc.connect(name='Autopilot')
    vessel = conn.space_center.active_vessel
    orbit = vessel.orbit
    ap = vessel.auto_pilot
    body = vessel.orbit.body
    srf_frame = vessel.surface_reference_frame
    obt_frame = vessel.orbit.body.non_rotating_reference_frame

    #Get a list of all engines and sort them in order of stage
    engine_list = vessel.parts.engines
    engine_stage_list = [e.part.stage for e in engine_list]
    sorted_engine_list = [e for s, e in sorted(zip(engine_stage_list, engine_list))] #Sort one array based on another - https://stackoverflow.com/questions/6618515/sorting-list-based-on-values-from-another-list
    sorted_engine_list.reverse() #Flip the order of the list so that engines used first are first in the list
    engine_stage_list.sort() #Make stage list match engine list order
    engine_stage_list.reverse()
    total_stages = conn.space_center.active_vessel.control.current_stage #Allows verification if rocket has been launched already

    #Build server dictionaries
    for key, value in flight_dict.items():
        flight_dict[key] = conn.add_stream(getattr, vessel.flight(srf_frame), key) #Setup flight attribute streams
    for key, value in ksc_dict.items():
        ksc_dict[key] = conn.add_stream(getattr, conn.space_center, key) #Setup flight attribute streams

##    while(True):
##        print(orbit.apoapsis)
##        time.sleep(0.5)

    def level_plane():
        roll_throw = 0.1
        cutoff_angle = 1
        #Keep plane level while changing headingaq
        if(flight_dict["roll"]() > cutoff_angle):
##            print("roll: " + str(flight_dict["roll"]))
            vessel.control.roll = -1*roll_throw
        elif(flight_dict["roll"]() < -1*cutoff_angle):
##            print("roll: " + str(flight_dict["roll"]))
            vessel.control.roll = roll_throw
        else:
            vessel.control.roll = 0

    def follow_heading(heading):
        cutoff_angle = 0.01
        yaw_throw = -0.2
        angle_diff = (flight_dict["heading"]() + flight_dict["sideslip_angle"]() - heading + 180 + 360) % 360 - 180
##        print("heading diff: " + str(angle_diff))
        if(abs(angle_diff) > cutoff_angle):
            vessel.control.yaw = angle_diff/abs(angle_diff)*yaw_throw
        else:
            vessel.control.yaw = 0
        level_plane()

    def calc_delta_v(target_radius): #https://krpc.github.io/krpc/tutorials/launch-into-orbit.html
        mu = body.gravitational_parameter
        r = orbit.apoapsis
        a1 = orbit.semi_major_axis
        a2 = target_radius
        v1 = math.sqrt(mu*((2./r)-(1./a1)))
        v2 = math.sqrt(mu*((2./r)-(1./a2)))
        delta_v = v2 - v1
        return delta_v

    def calc_burn_time(delta_v): #https://krpc.github.io/krpc/tutorials/launch-into-orbit.html
        F = vessel.available_thrust
        Isp = vessel.specific_impulse * 9.82
        m0 = vessel.mass
        m1 = m0 / math.exp(delta_v/Isp)
        flow_rate = F / Isp
        burn_time = (m0 - m1) / flow_rate
        return burn_time

################################################

    def launch():
        ap.sas = True
        vessel.control.throttle = 0.9
        time.sleep(2)
        gui_pipe.send("Launching off of " + str(body.name) + "!")
        vessel.control.activate_next_stage()
        time.sleep(0.1)
        vessel.control.activate_next_stage()
        time.sleep(0.1)
        ap.sas_mode = ap.sas_mode.prograde
        ap.target_heading = 90

    def stage_at_empty(offset):
        cur_stage = conn.space_center.active_vessel.control.current_stage+1
        for s, e in zip(engine_stage_list, sorted_engine_list):
            print(str(cur_stage) + " " + str(s))
            if(s == cur_stage):
##                  try:
                    while e.part.engine.has_fuel and kill_flag.value == 0:
                        time.sleep(0.1)
                        follow_heading(90)
##                  except:
                    print("Engine is already gone")
        vessel.control.yaw = 0
        vessel.control.roll = 0
        vessel.control.activate_next_stage()
        gui_pipe.send("Stage separation complete")

    def climb_to_orbit(offset):
        target_alt = body.equatorial_radius + body.atmosphere_depth + offset
        while(orbit.apoapsis < target_alt and kill_flag.value == 0):
            time.sleep(0.1)
##            print(str(orbit_dict["apoapsis"]) + " " + str(target_alt))
##            follow_heading(90)
        vessel.control.throttle = 0
        gui_pipe.send("Apoapsis is at target altitude")
        conn.space_center.physics_warp_factor = 3
        while(flight_dict["mean_altitude"]() <= body.atmosphere_depth + 100):
            time.sleep(1)

    def circularize_orbit_to_apoapsis():
        delta_v = calc_delta_v(orbit.apoapsis)
        node = vessel.control.add_node(ksc_dict["ut"]() + vessel.orbit.time_to_apoapsis, prograde=delta_v) #Add navigation node
        print("Delta-v node added")
        burn_time = calc_burn_time(delta_v)
        burn_ut = ksc_dict["ut"]() + orbit.time_to_apoapsis - (burn_time/2.)
        lead_time = 5
        conn.space_center.warp_to(burn_ut - lead_time)
        while orbit.time_to_apoapsis - (burn_time/2.) > 0 and kill_flag.value == 0:
            time.sleep(0.1)
        vessel.control.throttle = 1.0
        time.sleep(burn_time - 0.1)
        vessel.control.throttle = 0
        node.remove()

    def deploy_parts():
        antenna_list = vessel.parts.antennas
        for antenna in antenna_list:
            if(antenna.deployable):
                antenna.deployed = True

    def hohmann_transfer(body): #https://github.com/whatdamath/KerbalSpaceProgram/blob/master/kerbalMoonTransfer.py
        obt_frame = vessel.orbit.body.non_rotating_reference_frame
        target_orbit = conn.space_center.bodies[body].position(vessel.orbit.body.non_rotating_reference_frame)
        target_radius = ((target_orbit[0]**2 + target_orbit[2]**2)**0.5)
        angle = 10
        angular_diff = 15
        orbit_angular_velocity = 2*math.pi/orbit.period #Angular velocity in rad/sec

        while True and kill_flag.value == 0:
            vessel_orbit = vessel.position(obt_frame)
            target_orbit = conn.space_center.bodies[body].position(vessel.orbit.body.non_rotating_reference_frame)
            target_radius = ((target_orbit[0]**2 + target_orbit[2]**2)**0.5)
            delta_v = calc_delta_v(target_radius)
            burn_time = calc_burn_time(delta_v)

            angular_diff = math.pi*((1-(1/(2*math.sqrt(2)))*math.sqrt((orbit.semi_major_axis/target_radius+1)**3)))

            #phase angle
            dot = target_orbit[0] * vessel_orbit[0] + target_orbit[2] * vessel_orbit[2]
            det = target_orbit[0] * vessel_orbit[2] - vessel_orbit[0] * target_orbit[2]
            angle = math.atan2(det, dot)
            target_phase_angle = angle + orbit_angular_velocity*burn_time/2*0

            print("Angle Difference: " + str(abs(target_phase_angle+angular_diff)))
            if(abs(target_phase_angle+angular_diff) <= 0.01):
                break

        vessel.control.throttle = 1.0
        time.sleep(burn_time - 0.1)
        vessel.control.throttle = 0
        time.sleep(0.1)

    def execute_maneuver_nodes():
        gui_pipe.send("Executing Node(s)")
        node_list = vessel.control.nodes
        for node in node_list:
            delta_v = node.delta_v
            burn_time = calc_burn_time(delta_v)
            burn_ut = node.ut - (burn_time/2.)
            lead_time = 5
            conn.space_center.warp_to(burn_ut - lead_time)
            while ksc_dict["ut"]() < burn_ut and kill_flag.value == 0:
                time.sleep(0.1)
            vessel.control.throttle = 1.0
            time.sleep(abs(burn_time - 0.1))
            vessel.control.throttle = 0
            node.remove()
        gui_pipe.send("All nodes complete")

    def launch_sequence():
        gui_pipe.send("Launching")
        launch()
        stage_at_empty(1)
        vessel.control.throttle = 1
        stage_at_empty(1)
        climb_to_orbit(2000)
        gui_pipe.send("Circularizing Orbit")
        circularize_orbit_to_apoapsis()
        gui_pipe.send("Orbit Circularized")
        deploy_parts()
        gui_pipe.send("Parts Deployed - Launch Complete")


    dispatch_dict = {"Launch": launch_sequence, "Nodes": execute_maneuver_nodes}

    while kill_flag.value == 0:
        if(gui_pipe.poll(0.1)):
            func_string = gui_pipe.recv()
            dispatch_dict[func_string]()

    print("Autopilot stream fail")

def spam_science(gui_pipe, kill_flag):
    def run_exp():
        for exp in vessel.parts.experiments:
            exp_part = exp.part
            print(str(exp_part.name) + " " + str(exp.available))
            if (exp.available and len(exp.data) == 0):
                exp.run()
                if(kill_flag.value != 0):
                    break
                time.sleep(2)
            for data in  exp.data:
                print("Transmit: " + str(data.transmit_value) + ", Value: " + str(data.science_value) + ", Rerun: " + str(exp.rerunnable))
                if(kill_flag.value != 0):
                    break
                if(data.transmit_value > 0 and exp.rerunnable):
                    exp.transmit()
                elif(data.science_value > 0):
                    pass
                else:
                    exp.dump()
            exp.reset()

    prev_biome = None
    conn = krpc.connect(name='Science stream')
    vessel = conn.space_center.active_vessel
    science_on = False

    while(kill_flag.value == 0):
        if(gui_pipe.poll(0.1)):
            science_on = gui_pipe.recv()
        if(vessel.biome != prev_biome):
            print("Transition biome: " + str(prev_biome) + " to " + vessel.biome)
            prev_biome = vessel.biome
            if(science_on):
                run_exp()

def GUI(stream_flight_dict, stream_vessel_dict, stream_orbit_dict, stream_body_dict, stream_ksc_dict):
        def Launch():
            gui_to_auto.send("Launch")

        def Execute_Nodes():
            gui_to_auto.send("Nodes")

        def Spam_Science():
            cur_text = button_text["Spam Science"].get()
            if(cur_text == "Spam Science"):
                gui_to_science.send(True)
                button_text["Spam Science"].set("Stop Science")
            else:
                gui_to_science.send(False)
                button_text["Spam Science"].set("Spam Science")

        def Suicide_Burn():
            gui_to_auto.send("Suicide_Burn")

        def Circularize_to_Periapsis():
            gui_to_auto.send("Circularize_to_Periapsis")

        def update():
            connArray = []
            if(gui_to_auto.poll(0.1)):
                reply = gui_to_auto.recv()
                flight_status_label['text'] = "Flight: " + str(reply)
            if(gui_to_science.poll(0.1)):
                reply = gui_to_science.recv()
                science_status_label['text'] = "Science: " + str(reply)

            master.after(1000, update) #Calls the function again after delay in GUI main loop


        def kill_process():
            kill_flag.value = 1
##            server_process.terminate()
            autopilot_process.terminate()
            science_process.terminate()
##            server_process.join() #Verify that process is terminated
            autopilot_process.join()
            science_process.join()

        def Quit():
            kill_process()
            master.destroy()
            sys.exit()

        #Start data stream
        kill_flag = Value('i', 0)

        #Build pipes
        gui_to_auto, auto_to_gui = Pipe()
        gui_to_science, science_to_gui = Pipe()

        #Build processes
##        server_process = Process(target=server_stream, args=(stream_flight_dict, stream_vessel_dict, stream_orbit_dict, stream_body_dict, stream_ksc_dict, kill_flag))
        autopilot_process = Process(target=auto_pilot, args=(stream_flight_dict, stream_ksc_dict, auto_to_gui, kill_flag))
        science_process = Process(target=spam_science, args=(science_to_gui, kill_flag))

##        try:
##        server_process.start()
        autopilot_process.start()
        science_process.start()

##        except KeyboardInterrupt:
##            kill_process()
##            master.destroy()
##            sys.exit()

        #Make GUI
        master = Tk()
        style = Style()
        style.configure('TButton', font =('Lucida Grande', 20, 'bold'), borderwidth = '4')
        style.map('TButton', foreground = [('active', '!disabled', 'green')], background = [('active', 'black')])
        label_font_style = tkFont.Font(family="Lucida Grande", size=16) #Font for labels

        master.title("Ship Control")

        #Create master set of buttons
        button_func = OrderedDict([("Launch", Launch), ("Execute Nodes", Execute_Nodes), ("Spam Science", Spam_Science), ("Suicide Burn", Suicide_Burn), ("Circularize to Periapsis", Circularize_to_Periapsis), ("Quit", Quit)])
        button_dict = OrderedDict()
        button_text = OrderedDict()
        grid_index = 0
        for key, func in button_func.items():
            button_text[key] = StringVar()
            button_dict[key] = Button(master, textvariable=button_text[key], command=func)
            button_text[key].set(key)
            button_dict[key].grid(column=0, row=grid_index, padx=10, pady=10)
            grid_index += 1
        flight_status_label = Label(master, text = "Flight: On Pad", font = label_font_style)
        flight_status_label.grid(column=0, row=grid_index, padx=10, pady=10)
        grid_index += 1
        science_status_label = Label(master, text = "Science: Science Off", font = label_font_style)
        science_status_label.grid(column=0, row=grid_index, padx=10, pady=10)
        update()
        master.mainloop()



def main():
    stream_flight_dict = dict()
    stream_vessel_dict = dict()
    stream_orbit_dict = dict()
    stream_body_dict = dict()
    stream_ksc_dict = dict()

    #Declare variables to be retrieved via server streaming
    #Flight Variables - Used to get flight telemetry for a vessel, by calling Vessel.flight(). All of the information returned by this class is given in the reference frame passed to that method. Obtained by calling Vessel.flight().
    stream_flight_dict["g_force"] = 0.0 #The current G force acting on the vessel in g
    stream_flight_dict["mean_altitude"] = 0.0 #altitude above sea-level
    stream_flight_dict["surface_altitude"] = 0.0 #altitude above surface or ocean
    stream_flight_dict["bedrock_altitude"] = 0.0 #altitude above surface or bedrock below ocean
    stream_flight_dict["elevation"] = 0.0 #The elevation of the terrain under the vessel, in meters. This is the height of the terrain above sea level, and is negative when the vessel is over the sea.
    stream_flight_dict["latitude"] = 0.0 #The latitude of the vessel for the body being orbited, in degrees.
    stream_flight_dict["longitude"] = 0.0 #The longitude of the vessel for the body being orbited, in degrees.
    stream_flight_dict["velocity"] = None #The velocity of the vessel, in the reference frame ReferenceFrame.
    stream_flight_dict["speed"] = 0.0 #The speed of the vessel in meters per second, in the reference frame ReferenceFrame.
    stream_flight_dict["horizontal_speed"] = 0.0 #The horizontal speed of the vessel in meters per second, in the reference frame
    stream_flight_dict["vertical_speed"] = 0.0 #The vertical speed of the vessel in meters per second, in the reference frame ReferenceFrame.
    stream_flight_dict["center_of_mass"] = None #The position of the center of mass of the vessel, in the reference frame ReferenceFrame
    stream_flight_dict["rotation"] = None #The rotation of the vessel, in the reference frame ReferenceFrame
    stream_flight_dict["direction"] = None #The direction that the vessel is pointing in, in the reference frame ReferenceFrame.
    stream_flight_dict["pitch"] = 0.0 #The pitch of the vessel relative to the horizon, in degrees. A value between -90° and +90°.
    stream_flight_dict["heading"] = 0.0 #The heading of the vessel (its angle relative to north), in degrees. A value between 0° and 360°.
    stream_flight_dict["roll"] = 0.0 #The roll of the vessel relative to the horizon, in degrees. A value between -180° and +180°.
    stream_flight_dict["prograde"] = None #The prograde direction of the vessels orbit, in the reference frame ReferenceFrame.
    stream_flight_dict["retrograde"] = None #The retrograde direction of the vessels orbit, in the reference frame ReferenceFrame.
    stream_flight_dict["normal"] = None #The direction normal to the vessels orbit, in the reference frame ReferenceFrame.
    stream_flight_dict["anti_normal"] = None #The direction opposite to the normal of the vessels orbit, in the reference frame ReferenceFrame.
    stream_flight_dict["radial"] = None #The radial direction of the vessels orbit, in the reference frame ReferenceFrame.
    stream_flight_dict["anti_radial"] = None #The direction opposite to the radial direction of the vessels orbit, in the reference frame ReferenceFrame.
    stream_flight_dict["atmosphere_density"] = 0.0 #The current density of the atmosphere around the vessel, in kg/m3.
    stream_flight_dict["dynamic_pressure"] = 0.0 #The dynamic pressure acting on the vessel, in Pascals. This is a measure of the strength of the aerodynamic forces. It is equal to 12.air density.velocity2. It is commonly denoted Q.
    stream_flight_dict["static_pressure"] = 0.0 #The static atmospheric pressure acting on the vessel, in Pascals.
    stream_flight_dict["static_pressure_at_msl"] = 0.0 #The static atmospheric pressure at mean sea level, in Pascals.
    stream_flight_dict["aerodynamic_force"] = None #The total aerodynamic forces acting on the vessel, in reference frame ReferenceFrame.
    stream_flight_dict["lift"] = None #The aerodynamic lift currently acting on the vessel.
    stream_flight_dict["drag"] = None #The aerodynamic drag currently acting on the vessel.
    stream_flight_dict["speed_of_sound"] = 0.0 #The speed of sound, in the atmosphere around the vessel, in m/s.
    stream_flight_dict["mach"] = 0.0 #The speed of the vessel, in multiples of the speed of sound.
    stream_flight_dict["true_air_speed"] = 0.0 #The true air speed of the vessel, in meters per second.
    stream_flight_dict["equivalent_air_speed"] = 0.0 #The equivalent air speed of the vessel, in meters per second.
    stream_flight_dict["terminal_velocity"] = 0.0 #An estimate of the current terminal velocity of the vessel, in meters per second. This is the speed at which the drag forces cancel out the force of gravity.
    stream_flight_dict["angle_of_attack"] = 0.0 #The pitch angle between the orientation of the vessel and its velocity vector, in degrees.
    stream_flight_dict["sideslip_angle"] = 0.0 #The yaw angle between the orientation of the vessel and its velocity vector, in degrees.
    stream_flight_dict["total_air_temperature"] = 0.0 #The total air temperature of the atmosphere around the vessel, in Kelvin. This includes the Flight.static_air_temperature and the vessel’s kinetic energy.
    stream_flight_dict["static_air_temperature"] = 0.0 #The static (ambient) temperature of the atmosphere around the vessel, in Kelvin.

    #(requires Ferram)
    if(False):
        stream_flight_dict["reynolds_number"] = 0.0 #The vessels Reynolds number.
        stream_flight_dict["stall_fraction"] = 0.0 #The current amount of stall, between 0 and 1. A value greater than 0.005 indicates a minor stall and a value greater than 0.5 indicates a large-scale stall.
        stream_flight_dict["drag_coefficient"] = 0.0 #The coefficient of drag. This is the amount of drag produced by the vessel. It depends on air speed, air density and wing area.
        stream_flight_dict["lift_coefficient"] = 0.0 #The coefficient of lift. This is the amount of lift produced by the vessel, and depends on air speed, air density and wing area.
        stream_flight_dict["ballistic_coefficient"] = 0.0 #The ballistic coefficient.
        stream_flight_dict["thrust_specific_fuel_consumption"] = 0.0 #The thrust specific fuel consumption for the jet engines on the vessel. This is a measure of the efficiency of the engines, with a lower value indicating a more efficient vessel. This value is the number of Newtons of fuel that are burned, per hour, to produce one newton of thrust.

    #############################

    #Vessel Variables - These objects are used to interact with vessels in KSP. This includes getting orbital and flight data, manipulating control inputs and managing resources. Created using active_vessel or vessels.
    stream_vessel_dict["biome"] = None #The name of the biome the vessel is currently in.
##    stream_vessel_dict["bounding_box"] = manager.list([0]*6) #The axis-aligned bounding box of the vessel in the given reference frame.
    stream_vessel_dict["thrust"] = 0.0 #The total thrust currently being produced by the vessel’s engines, in Newtons. This is computed by summing Engine.thrust for every engine in the vessel.
    stream_vessel_dict["available_thrust"] = 0.0 #Gets the total available thrust that can be produced by the vessel’s active engines, in Newtons. This is computed by summing Engine.available_thrust for every active engine in the vessel.
    stream_vessel_dict["max_thrust"] = 0.0 #The total maximum thrust that can be produced by the vessel’s active engines, in Newtons. This is computed by summing Engine.max_thrust for every active engine.
    stream_vessel_dict["specific_impulse"] = 0.0 #The combined specific impulse of all active engines, in seconds. This is computed using the formula described here.
    stream_vessel_dict["mass"] = 0.0 #The total mass of the vessel, including resources, in kg.

    ###############################

    #Orbit Variables
    stream_orbit_dict["apoapsis"] = None #Gets the apoapsis of the orbit, in meters, from the center of mass of the body being orbited.
    stream_orbit_dict["periapsis"] = None #The periapsis of the orbit, in meters, from the center of mass of the body being orbited.
    stream_orbit_dict["apoapsis_altitude"] = None #The apoapsis of the orbit, in meters, above the sea level of the body being orbited.
    stream_orbit_dict["periapsis_altitude"] = None #The periapsis of the orbit, in meters, above the sea level of the body being orbited.
    stream_orbit_dict["semi_major_axis"] = None #The semi-major axis of the orbit, in meters.
    stream_orbit_dict["semi_minor_axis"] = None #The semi-minor axis of the orbit, in meters.
    stream_orbit_dict["radius"] = None #The current radius of the orbit, in meters. This is the distance between the center of mass of the object in orbit, and the center of mass of the body around which it is orbiting.
    stream_orbit_dict["speed"] = None #The current orbital speed of the object in meters per second.
    stream_orbit_dict["period"] = None #The orbital period, in seconds.
    stream_orbit_dict["time_to_apoapsis"] = None #The time until the object reaches apoapsis, in seconds.
    stream_orbit_dict["time_to_periapsis"] = None #The time until the object reaches periapsis, in seconds.
    stream_orbit_dict["eccentricity"] = None #The eccentricity of the orbit.
    stream_orbit_dict["inclination"] = None #The inclination of the orbit, in radians.
    stream_orbit_dict["longitude_of_ascending_node"] = None #The longitude of the ascending node, in radians.
    stream_orbit_dict["argument_of_periapsis"] = None #The argument of periapsis, in radians.
    stream_orbit_dict["mean_anomaly_at_epoch"] = None #The mean anomaly at epoch.
    stream_orbit_dict["epoch"] = None #The time since the epoch (the point at which the mean anomaly at epoch was measured, in seconds.
    stream_orbit_dict["mean_anomaly"] = None #The mean anomaly.
    stream_orbit_dict["eccentric_anomaly"] = None #The eccentric anomaly.
    stream_orbit_dict["true_anomaly"] = None #The true anomaly.
    stream_orbit_dict["orbital_speed"] = None #The current orbital speed in meters per second.
    stream_orbit_dict["time_to_soi_change"] = None #The time until the object changes sphere of influence, in seconds. Returns NaN if the object is not going to change sphere of influence.
    stream_orbit_dict["next_orbit"] = None #If the object is going to change sphere of influence in the future, returns the new orbit after the change. Otherwise returns None.

    ######################################

    #Body Variables
    stream_body_dict["name"] = None #The name of the body.
    stream_body_dict["mass"] = None #The mass of the body, in kilograms.
    stream_body_dict["gravitational_parameter"] = None #The standard gravitational parameter of the body in m3s−2.
    stream_body_dict["surface_gravity"] = None #The acceleration due to gravity at sea level (mean altitude) on the body, in m/s2.
    stream_body_dict["rotational_period"] = None #The sidereal rotational period of the body, in seconds.
    stream_body_dict["rotational_speed"] = None #The rotational speed of the body, in radians per second.
    stream_body_dict["rotation_angle"] = None #The current rotation angle of the body, in radians. A value between 0 and 2π
    stream_body_dict["initial_rotation"] = None #The initial rotation angle of the body (at UT 0), in radians. A value between 0 and 2π
    stream_body_dict["equatorial_radius"] = None #The equatorial radius of the body, in meters.
    stream_body_dict["sphere_of_influence"] = None #The radius of the sphere of influence of the body, in meters.
    stream_body_dict["has_atmosphere"] = None #True if the body has an atmosphere.
    stream_body_dict["atmosphere_depth"] = None #The depth of the atmosphere, in meters.
    stream_body_dict["has_atmospheric_oxygen"] = None #has_atmospheric_oxygen
    stream_body_dict["biomes"] = None #The biomes present on this body.
    stream_body_dict["flying_high_altitude_threshold"] = None #The altitude, in meters, above which a vessel is considered to be flying “high” when doing science.
    stream_body_dict["space_high_altitude_threshold"] = None #The altitude, in meters, above which a vessel is considered to be in “high” space when doing science.

    ###########################################

    #Kerbal space center variables
    stream_ksc_dict["ut"] = 0.0 #The current universal time in seconds.

    ##########################################
##    auto_pilot(stream_flight_dict, stream_ksc_dict)
    GUI(stream_flight_dict, stream_vessel_dict, stream_orbit_dict, stream_body_dict, stream_ksc_dict)

if __name__ == '__main__':
    main()
