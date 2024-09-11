# Semi-Autonomous-RC-Car

## Description

I developed and designed an RC car from scratch, incorporating semi-autonomous features such as adaptive cruise control and low-speed emergency braking. Driven by a passion for software development and automotive engineering, I integrated these interests into a project that aims to advance the capabilities of remote-controlled vehicles.

The RC car is built around an Arduino Nano microcontroller. For steering, I implemented a servo motor at the front wheel shaft and for the movement, I used 2 DC-encoded motors controlled using a L293D quadruple half-H driver. I designed and tested different prototypes on AutoCad and printed them using a 3D printer.

I used an SPI communication protocol on nRF24L01 modules, allowing seamless communication with the controller and the car. The controller has two joysticks for manual control of the car’s movement. For the Advanced Driver Assistance System (ADAS) features, I used an  HC-SR04 ultrasonic sensor at the front of the car to detect the distance of objects in front of the car which allowed me to implement low-speed braking and adaptive cruise control.
 an HC-SR04 ultrasonic sensor is employed to enable adaptive cruise control and low-speed emergency braking.


## Table of Contents (Optional)

If your README is long, add a table of contents to make it easy for users to find what they need.

- [Installation](#installation)
- [Usage](#usage)
- [Credits](#credits)
- [License](#license)

## Installation

What are the steps required to install your project? Provide a step-by-step description of how to get the development environment running.

## Usage

Provide instructions and examples for use. Include screenshots as needed.

To add a screenshot, create an `assets/images` folder in your repository and upload your screenshot to it. Then, using the relative filepath, add it to your README using the following syntax:

    ```md
    ![alt text](assets/images/screenshot.png)
    ```

## Credits

List your collaborators, if any, with links to their GitHub profiles.

If you used any third-party assets that require attribution, list the creators with links to their primary web presence in this section.

If you followed tutorials, include links to those here as well.

## License

The last section of a high-quality README file is the license. This lets other developers know what they can and cannot do with your project. If you need help choosing a license, refer to [https://choosealicense.com/](https://choosealicense.com/).

---

🏆 The previous sections are the bare minimum, and your project will ultimately determine the content of this document. You might also want to consider adding the following sections.

## Badges

![badmath](https://img.shields.io/github/languages/top/lernantino/badmath)

Badges aren't necessary, per se, but they demonstrate street cred. Badges let other developers know that you know what you're doing. Check out the badges hosted by [shields.io](https://shields.io/). You may not understand what they all represent now, but you will in time.

## Features

If your project has a lot of features, list them here.

## How to Contribute

If you created an application or package and would like other developers to contribute it, you can include guidelines for how to do so. The [Contributor Covenant](https://www.contributor-covenant.org/) is an industry standard, but you can always write your own if you'd prefer.

## Tests
