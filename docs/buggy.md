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
For the motor I settled with the Turnigy SK3 2836 1040 from [HobbyKing](https://hobbyking.com/en_us/turnigy-aerodrive-sk3-2836-1040kv-brushless-outrunner-motor.html?gclsrc=aw.ds&gclid=CjwKCAjwwvfrBRBIEiwA2nFiPUKxThyzLSzGWZayfOwnxFth31bcAuTGR65S_pAo1QQNIYw3izbd0RoCejoQAvD_BwE&___store=en_us) This is a brushless outrunner motor with good power and lowest kV that I could find. Meaning it runs well also with lower RPMs. It also has a can diameter of 28mm, which means it will just fit into the car, whereas all the 1/10 RC motors with their 36mm can diameter will not. 

When I first got the motor and hooked it up to the original car's ESC I found it was not suitable either, as it required some throttle to get started and could not run on low speeds. Although the high speed was less than the original car in-runner motor. After reading that non-sensored brushless motors would not have enough passive control through a standard sensorless ESC I initially gave up on the plan to use that motor and went with a brushed motor/ESC combination instead initially. However, I found that after changing the ESC I could run the motor very well at low speeds in a very controllable way. I am using a very cheap ESC from [Ebay](https://www.ebay.co.uk/itm/45-120A-ESC-Sensored-Brushless-Speed-Controller-for-RC-1-8-1-10-Car-Crawler/123836633866?ssPageName=STRK%3AMEBIDX%3AIT&var=424542043077&_trksid=p2057872.m2749.l2649). It turns out that this works brilliantly in the combination with the SK3 outrunner motor

