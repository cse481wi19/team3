/*
  JAVASCRIPT SECTION
  Here is where you add the code that controls the behavior of the element.
*/ 
/**
 * @customElement
 * @polymer
 */
class WebTeleopApp extends Polymer.Element {
  ready() {
    super.ready();
    this.hostname = window.location.hostname;
  }
  static get is() { return 'web-teleop-app'; }
  // List properties here, which we will not use in this lab!!!
  static get properties() {
    return {
    };
  }

  _handleConnection() {
    this.status = 'Connected to the websocket server.';
    console.log(this.status);
  }

  _handleClose() {
    this.status = 'Closed connection to the websocket server.';
    console.log(this.status);
  }

  _handleError() {
    this.status = 'Error connecting to the websocket server.';
    console.log(this.status);
  }

  // Rounds val to a given precision, where the precision is given as the    
  // step size between numbers in the output range.                          
  // E.g., _round(0.053, 0.1) = 0.1                                          
  // E.g., _round(0.053, 0.01) = 0.05                                        
  // E.g., _round(0.053, 0.001) = 0.053                                      
  _round(val, precision) {                                                   
    return Math.round(val/precision) * precision;                            
  }

  _setTorso() {
      this.status = 'Setting torso to ' + this.desiredTorsoHeight + ' meters...';
      this.$.torsoService.call({height: this.desiredTorsoHeight});
  }

  _handleTorsoSuccess() {
    this.status = 'Set torso to ' + this.desiredTorsoHeight + ' meters.';
  }

  _handleTorsoError(evt) {
    this.status = 'Error ' + evt.detail;
    console.log(this.status);
  }

  _startRight(evt) {
    evt.preventDefault(); // Prevent right-click menu from showing up after long press on mobile
    this.status = 'Driving right...';
    var baseTopic = this.$.baseTopic; // Get <ros-topic>
    if (this.baseCommand) {
      // The timer should not be set at this point, but clear it just in case
      clearInterval(this.baseCommand);
    }
    this.baseCommand = setInterval(function() {
      baseTopic.publish({
        linear: {
          x: 0, // Set positive or negative meters/s to drive
          y: 0,
          z: 0
        },
        angular: {
          x: 0,
          y: 0,
          z: -0.2 // Set rads/s to turn
        }
      });
    }, 10); // Run this function every 10ms, or approximately 100 times per second.
  }

  _startLeft(evt) {
    evt.preventDefault(); // Prevent right-click menu from showing up after long press on mobile
    this.status = 'Driving left...';
    var baseTopic = this.$.baseTopic; // Get <ros-topic>
    if (this.baseCommand) {
      // The timer should not be set at this point, but clear it just in case
      clearInterval(this.baseCommand);
    }
    this.baseCommand = setInterval(function() {
      baseTopic.publish({
        linear: {
          x: 0, // Set positive or negative meters/s to drive
          y: 0,
          z: 0
        },
        angular: {
          x: 0,
          y: 0,
          z: 0.2 // Set rads/s to turn
        }
      });
    }, 10); // Run this function every 10ms, or approximately 100 times per second.
  }
  _startBackward(evt) {
    evt.preventDefault(); // Prevent right-click menu from showing up after long press on mobile
    this.status = 'Driving backward...';
    var baseTopic = this.$.baseTopic; // Get <ros-topic>
    if (this.baseCommand) {
      // The timer should not be set at this point, but clear it just in case
      clearInterval(this.baseCommand);
    }
    this.baseCommand = setInterval(function() {
      baseTopic.publish({
        linear: {
          x: -0.1, // Set positive or negative meters/s to drive
          y: 0,
          z: 0
        },
        angular: {
          x: 0,
          y: 0,
          z: 0 // Set rads/s to turn
        }
      });
    }, 10); // Run this function every 10ms, or approximately 100 times per second.
  }

  _startForward(evt) {
    evt.preventDefault(); // Prevent right-click menu from showing up after long press on mobile
    this.status = 'Driving forward...';
    var baseTopic = this.$.baseTopic; // Get <ros-topic>
    if (this.baseCommand) {
      // The timer should not be set at this point, but clear it just in case
      clearInterval(this.baseCommand);
    }
    this.baseCommand = setInterval(function() {
      baseTopic.publish({
        linear: {
          x: 0.1, // Set positive or negative meters/s to drive
          y: 0,
          z: 0
        },
        angular: {
          x: 0,
          y: 0,
          z: 0 // Set rads/s to turn
        }
      });
    }, 10); // Run this function every 10ms, or approximately 100 times per second.
  }

  _endBaseCommand(evt) {
    this.status = 'Stopped driving.';
    clearInterval(this.baseCommand);
  }

  //
  // Gripper controls
  //
  _setGripper() {
      this.$.gripperService.call({});
  }

  //
  // Arm controls
  //
  _setArm() {
      this.$.armService.call({});
  }

  //
  // Head Pan/Tilt
  //

  _setTilt() {
      this.$.headService.call({pan: 0.0, tilt: this.desiredTiltHeight});
  }
}

window.customElements.define(WebTeleopApp.is, WebTeleopApp);
