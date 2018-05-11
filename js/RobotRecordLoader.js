function loadRobotRecord(data) {
    let recorded_file = data.recorded_file
    let control_gui = data.control_gui || false 
    let t_f = data.fading || 0.5 
    let t_d = data.delay || 0.5 
    let clock = data.clock
    let mixer = data.mixer

    new THREE.FileLoader().load( recorded_file, function ( animation ) {
        let track = THREE.AnimationClip.parse(JSON.parse(animation)); 
        let action = mixer.clipAction( track );   
        if (control_gui == true) {
            let gui = new dat.GUI();  
            clipControl( gui, action, mixer, track, t_f, t_d );
        } else{
            action.reset().startAt( mixer.time + t_d ).fadeIn( t_f ).play();
        }
    })

}


var clipControl = function  clipControl( gui, action, mixer, track, t_f, t_d) {
    var folder = gui.addFolder( "Clip '" + track.name + "'" ),
        API = {
            'play()': function play() {
                action.play();
            },
            'stop()': function() {
                action.stop();
            },
            'reset()': function() {
                action.reset();
            },
            get 'time ='() {
                return action !== null ? action.time : 0;
            },
            set 'time ='( value ) {
                action.time = value;
            },
            get 'paused ='() {
                return action !== null && action.paused;
            },
            set 'paused ='( value ) {
                action.paused = value;
            },
            get 'enabled ='() {
                return action !== null && action.enabled;
            },
            set 'enabled ='( value ) {
                action.enabled = value;
            },
            'play delayed': function() {
                action.startAt( mixer.time + t_d ).play();
            },
            'fade in': function() {
                action.reset().fadeIn( t_f ).play();
            },
            'fade out': function() {
                action.fadeOut( t_f ).play();
            },

            get 'loop mode'() {
                return action !== null ? action.loop : THREE.LoopRepeat;
            },
            set 'loop mode'( value ) {
                action.loop = + value;
            },

            get 'local root'() { return rootName; },
            set 'local root'( value ) {
                rootName = value;
        }
    };
    folder.add( API, 'play()' );
    folder.add( API, 'stop()' );
    folder.add( API, 'reset()' );
    folder.add( API, 'time =', 0, track.duration ).listen();
    folder.add( API, 'paused =' ).listen();
    folder.add( API, 'enabled =' ).listen();
    folder.add( API, 'play delayed' );
    folder.add( API, 'fade in' );
    folder.add( API, 'fade out' );
    folder.add( API, 'loop mode', {
        "LoopOnce": THREE.LoopOnce,
        "LoopRepeat": THREE.LoopRepeat,
        "LoopPingPong": THREE.LoopPingPong
    } );
    API[ 'fade in' ]();
    //folder.open();
}; 

