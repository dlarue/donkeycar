# Turnigy Brushless 4WD buggy

The Turnigy 1/16th 4WD Brushless Buggy is a well made RC car which comes at a 
reasonable price. The car can be bought from [HobbyKing](https://hobbyking.com/en_us/turnigy-1-16-brushless-4wd-racing-buggy-w-25a-power-system-and-2-4ghz-radio-rtr.html?gclid=CjwKCAjwqqrmBRAAEiwAdpDXtFS4SQOmtAFHIMZ9HrEiBeGwMeOg9UfNnKB6_Nr-yxoUUoyarf6FORoC7NgQAvD_BwE&gclsrc=aw.ds&___store=en_us). 

![4WD_Buggy](./assets/4WD_buggy.jpg)

I bought the buggy to build my donkey car on it. This is not as straight forward as building the _standard_ Exceed or Magnet:
* the car needs some modifications in order to be usable as a donkey car
* the modifications are an additional investment (probably more in time than in $)
* for that work you need to be happy to do some mechanical work, like drilling, using a file if you use metal, etc
* for making things neat and fitting, it's very handy to have access to a 3D printer

I am not familiar with the Exceed or Magnet car, but judging the pictures and the specs, I think that the Turnigy buggy is at least as good as the standard cars. Doing that built has been an interesting experience for me and I made many mistakes and took different approaches. Finally, I think I have now converged towards a great vehicle with high quality driving behaviour and a very robust architecture, so I'll share my build here. 

## Parts to chuck out
The buggy comes with a small ESC and quite powerful brushless motor. According the specs it is supposed to run up to 25mph. This is probably great for racing around in the backyard but not so suitable for running donkey on it as you will require good control at low speeds. I tried a couple of laps in the garden before I started the donkey car build and it was fun to drive but hard to control (as I am no RC driver). It was also quite noisy, and the motor as well as the ESC reached high temperatures in action.

## Parts to buy
For the motor I settled with the Turnigy SK3 2836 1040 from [HobbyKing](https://hobbyking.com/en_us/turnigy-aerodrive-sk3-2836-1040kv-brushless-outrunner-motor.html?gclsrc=aw.ds&gclid=CjwKCAjwwvfrBRBIEiwA2nFiPUKxThyzLSzGWZayfOwnxFth31bcAuTGR65S_pAo1QQNIYw3izbd0RoCejoQAvD_BwE&___store=en_us) This is a brushless outrunner motor with good power and lowest kV that I could find. Meaning it runs well also with lower RPMs. It also has a can diameter of 28mm, which means it will just fit into the car, whereas all the 1/10 RC motors with their 36mm can diameter will not. In order to drive the spur gear there is also a motor pinion for a 3.175 shaft required. I chose 14 teeth, 15 will fit, too. This is a standard RC part that can be bought in any RC online shop.
![SK3 outrunner](./assets/SK3-motor.jpg)

When I first got the motor and hooked it up to the original car's ESC I found it was not suitable either, as it required some throttle to get started and could not run on low speeds. Although the high speed was less than the original car in-runner motor. After reading that non-sensored brushless motors would not have enough passive control through a standard sensorless ESC I initially gave up on the plan to use that motor and went with a brushed motor/ESC combination instead initially. However, I found that after changing the ESC I could run the motor very well at low speeds in a very controllable way. I am using a very cheap ESC from [Ebay](https://www.ebay.co.uk/itm/45-120A-ESC-Sensored-Brushless-Speed-Controller-for-RC-1-8-1-10-Car-Crawler/123836633866?ssPageName=STRK%3AMEBIDX%3AIT&var=424542043077&_trksid=p2057872.m2749.l2649). It turns out that this works brilliantly in the combination with the SK3 outrunner motor.
![45 Amp ESC](./assets/esc-45amp.jpg)

## The build
### The chassis
In order to fit the motor into the original mounting bracket there is some filing or drilling required as the motor mount hole pattern of the larger SK3 motor does not fit the original holes. Here is a picture with only the chassis of the car with the motor mounted (wheels look like they are not parallel but this is somehow due to the iPhone picture optics...)

![Chassis](./assets/chassis-1.jpeg)

Here is the motor as seen from the side. In the background is my odometer disc that is mounted on the drive shaft.
![Motor mounted](./assets/motor-1.jpeg)

This is the steering servo. I had to replace it after the original servo broke. 
![Steering servo](./assets/front-servo.jpeg)

My donkey car has been involved in many crashless - mostly due reckless driving when on its own, ignoring physical obstacles and doing all kind of stubborn juvenile behaviour. Some hard crash broke the front suspension unit that I replaced w/ an upgraded metal suspension one. But long story short, donkey needs a front bumper to protect him from himself, so I made a 3D printed part (actually I made 2 parts, the first one not strong enough as donkey broke that one almost immediately, too).
![bumper1](./assets/bumper-1.jpeg)
Here is where the bumper is fixed on the upper end.
![bumper2](./assets/bumper-2.jpeg)

### The base
I decided to create a fixed base plate that is mounted to the chassis with strong joints. I also wanted a camera holder to be fixed to the frame but with the option to remove the camera from it or to remove the electronics without much hassle. Therefore I created a base plate and camera holder from 1mm sheet metal (mild steel) and a 3D-printed electronics carrier plate. The camera also has a 3D-printed case that is fixed to the holder with screws. Here are the parts:
![all-parts](./assets/all-parts.jpeg)

The base plate requires some amount of drilling hols for screws and cable openings. Here I already installed a rear aluminium angle bar for mounting the main switch and shut-down push button and screw terminals for the power distribution.
![base](./assets/base.jpeg)

This is the camera holder with a right angle slotted plate, to feed the camera cable into the housing.
![cam-holder](./assets/cam-holder.jpeg)

The electronic carrier was modelled in Fusion360, using CAD drawings for the RPi and the PCA9685 for mounting studs. It also takes the RC receiver with a push fit and a 3 Amp voltage step dow converter (push fit plus a small bit of hot glue).
![elec-carrier](./assets/elec-carrier.jpeg)

For the assembly I use 3mm hex cap head (i.e Allen) screws and beer. The screws are small and fiddly, so you need to be relaxed during assembly. I have a whole box in all lenghts between 6mm and 40mm.
![screws](./assets/screws.jpeg)

Mounting the camera holder with its angle bracket to the base plate.
![base+cam](./assets/base+cam.jpeg)

Mounting the on/off switch and connecting to terminals.
![switch](./assets/switch.jpeg)

This is the angle bracket with the switch and the small push button installed. The latter allows to shut-down the RPi through setting a specifc pin to ground. Please see [Andreas](http://www.sensorsiot.org/raspberry-pi-tricks/) page.
![panel](./assets/panel.jpeg)

To mount the ESC to the base plate I 3D printed as small holder with 2 holes for the screws.
![esc-holder](./assets/esc-holder.jpeg)

This is how it looks with the whole electronics mounted on the rear of the base plate.
![esc-mounted](./assets/esc-mounted.jpeg)


