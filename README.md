# Particle-RF24

## About

An implementation of [TMRh20's optimized RF24 Library](https://github.com/TMRh20/RF24) for the Particle Photon.

Used [Technobly's SparkCore-RF24 library](https://github.com/technobly/SparkCore-RF24) as a reference to make sure it is compatible with Particle Photon.

## Design Goals

* Fully API compatibility with [TMRh20's RF24 Library](https://github.com/TMRh20/RF24), can run [RF24's examples](https://github.com/TMRh20/RF24/tree/master/examples) without massive modification.
* Fully communicationable with other devices runing [TMRh20's RF24 Library](https://github.com/TMRh20/RF24).
* Fully compatible with [Particle Photon](https://www.particle.io/prototype#photon). (with Electron if I am able to test on the real device)
* Can be included from Particle Build Web IDE.

## Example Usage

Check firmware/examples folders. More examples are comming...

## Circuit Diagram

Comming soon...

## FAQ

### Q: Why not just fork [TMRh20's RF24 Library](https://github.com/TMRh20/RF24) and add device support for Particle Photon?

A: It is eaiser to port the RF library, because Particle Photon is kind of combination of Arduino and ARM. And it needs special folder structure to make library accessiable from Partible Build Web IDE.

### Q: What if my device is damaged because of this library?

A: 😱 I take no responsibility for any damage and/or injuries to you and/or devices caused by this library!!! Do not use this library in any killing robot 🤖️ or missile launcher 🚀 projects.

