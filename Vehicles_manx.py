# Vehicles_manx.py
# 
# Created:  Feb 2016, E. Botero
# Modified: Jan 2024, O.N. Afify

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
import numpy as np
from SUAVE.Core import Units, Data
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_mass
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_kv

# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def setup():
    
    base_vehicle = base_setup()
    configs = configs_setup(base_vehicle)
    
    return configs
    
def base_setup():
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    

    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'UAV'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties
    vehicle.mass_properties.takeoff         = 3.23 * Units.kg
    vehicle.mass_properties.operating_empty = 3.23 * Units.kg
    vehicle.mass_properties.max_takeoff     = 3.23 * Units.kg 

    # basic parameters
    vehicle.reference_area                    = 0.594 * Units.meter       
    vehicle.envelope.ultimate_load            = 2.0
    vehicle.envelope.limit_load               = 1.5
    vehicle.envelope.maximum_dynamic_pressure = 115.*1.25 * Units.pascals #Max q

    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------   

    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'

    wing.areas.reference         = vehicle.reference_area
    wing.spans.projected         = 3.0 * Units.meters
    wing.aspect_ratio            = (wing.spans.projected**2)/wing.areas.reference 
    wing.sweeps.quarter_chord    = 0.0 * Units.deg
    wing.symmetric               = True
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 0.6
    wing.vertical                = False
    wing.high_lift               = True 
    wing.dynamic_pressure_ratio  = 1.0
    wing.chords.mean_aerodynamic = wing.areas.reference/wing.spans.projected
    wing.twists.root             = 1.5 * Units.degrees
    wing.twists.tip              = -1.5 * Units.degrees
    wing.highlift                = False  
    wing.vertical                = False 
    wing.number_ribs             = 32.
    wing.number_end_ribs         = 2.
    wing.transition_x_upper      = 0.6
    wing.transition_x_lower      = 1.0
    wing.origin                  = [[3.0,0.0,0.0]]
    wing.aerodynamic_center      = [0.02064,0.0,0.0]

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------        

    wing = SUAVE.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'

    wing.aspect_ratio         = 20. 
    wing.sweeps.quarter_chord = 0 * Units.deg
    wing.thickness_to_chord   = 0.12
    wing.taper                = 1.0
    wing.areas.reference      = 0.095 * Units.meter
    wing.areas.wetted         = 2.0 * wing.areas.reference
    wing.areas.exposed        = 0.8 * wing.areas.wetted
    wing.areas.affected       = 0.6 * wing.areas.wetted       
    wing.spans.projected      = 0.68 * Units.meter
    wing.twists.root          = 0.0 * Units.degrees
    wing.twists.tip           = 0.0 * Units.degrees      

    wing.vertical                = False 
    wing.symmetric               = True
    wing.dynamic_pressure_ratio  = 0.9      
    wing.number_ribs             = 11.
    wing.chords.root             = wing.areas.reference/wing.spans.projected
    wing.chords.tip              = wing.areas.reference/wing.spans.projected
    wing.chords.mean_aerodynamic = wing.areas.reference/wing.spans.projected  
    wing.origin                  = [[10.,0.0,0.0]]
    wing.aerodynamic_center      = [0.035,0.0,0.0]

    # add to vehicle
    vehicle.append_component(wing)    

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Vertical_Tail()
    wing.tag = 'vertical_stabilizer'    

    wing.aspect_ratio         = 20.       
    wing.sweeps.quarter_chord = 0 * Units.deg
    wing.thickness_to_chord   = 0.12
    wing.taper                = 1.0
    wing.areas.reference      = 0.032 * Units.meter
    wing.spans.projected      = 0.216 * Units.meter

    wing.chords.root             = wing.areas.reference/wing.spans.projected
    wing.chords.tip              = wing.areas.reference/wing.spans.projected
    wing.chords.mean_aerodynamic = wing.areas.reference/wing.spans.projected
    wing.areas.wetted            = 2.0 * wing.areas.reference
    wing.areas.exposed           = 0.8 * wing.areas.wetted
    wing.areas.affected          = 0.6 * wing.areas.wetted    
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees  
    wing.origin                  = [[10.,0.0,0.0]]
    wing.aerodynamic_center      = [0.026,0.0,0.0]    
    wing.symmetric               = True          
    wing.vertical                = True 
    wing.t_tail                  = False
    wing.dynamic_pressure_ratio  = 1.0
    wing.number_ribs             = 7.

    # add to vehicle
    vehicle.append_component(wing)  
    
    #------------------------------------------------------------------
    # Propulsor
    #------------------------------------------------------------------

    # build network
    net = Solar_Low_Fidelity()
    net.number_of_engines = 2.

    # Component 1 the Sun
    sun = SUAVE.Components.Energy.Processes.Solar_Radiation()
    net.solar_flux = sun

    # Component 2 the solar panels
    panel = SUAVE.Components.Energy.Converters.Solar_Panel()
    panel.ratio                = 0.9
    panel.area                 = vehicle.reference_area * panel.ratio 
    panel.efficiency           = 0.25
    panel.mass_properties.mass = panel.area*(0.60 * Units.kg)
    net.solar_panel            = panel

    # Component 3 the ESC
    esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency = 0.95 # Gundlach for brushless motors
    esc.current = 30 * Units.amps
    net.esc        = esc

    # Component 4 the Propeller
    prop = SUAVE.Components.Energy.Converters.Propeller_Lo_Fid()
    prop.propulsive_efficiency = 0.825
    net.propeller        = prop
    
    # Component 5 the Motor
    motor = SUAVE.Components.Energy.Converters.Motor_Lo_Fid()
    motor.speed_constant       = 1150. * Units['rpm/volt'] # RPM/volt is standard
    motor                      = size_from_kv(motor)    
    motor.gear_ratio           = 1. # Gear ratio, no gearbox
    motor.gearbox_efficiency   = 1. # Gear box efficiency, no gearbox
    motor.motor_efficiency     = 0.8;
    net.motor                  = motor    

    # Component 6 the Payload
    payload = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw           = 0. #Watts 
    payload.mass_properties.mass = 1.0 * Units.kg
    net.payload                  = payload

    # Component 7 the Avionics
    avionics = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 10. #Watts  
    net.avionics        = avionics      

    # Component 8 the Battery
    bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
    bat.mass_properties.mass = 1.186  * Units.kg
    bat.specific_energy      = 187.18 *Units.Wh/Units.kg
    bat.resistance           = 0.003
    initialize_from_mass(bat,bat.mass_properties.mass)
    net.battery              = bat

    # add the UAV network to the vehicle
    vehicle.append_component(net)  

    return vehicle

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    
    # ------------------------------------------------------------------
    #   Only One Configurations
    # ------------------------------------------------------------------

    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)

    return configs
