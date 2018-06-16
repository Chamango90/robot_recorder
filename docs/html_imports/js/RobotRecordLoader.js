var LoadRobotRecord = function(data) {

    let animation = data.animation
    this.viewer = data.viewer
    this.t_f = data.fading || 0.5 
    this.t_d = data.delay || 0.5 

    this.clock = new THREE.Clock();
    this.mixer = new THREE.AnimationMixer( viewer.world )
    this.gui = new dat.GUI();  
    this.track = THREE.AnimationClip.parse(JSON.parse(animation)); 
    this.action = this.mixer.clipAction( this.track );   
    this.recording = false;
    this.action.startAt( this.mixer.time + this.t_d ).fadeIn( this.t_f ).play();
    this.action.paused = true

    var animate = function(){
        requestAnimationFrame(animate)
        this.mixer.update(this.clock.getDelta() );
        this.viewer._dirty = true; // trigger update of urdf-viewer
        if (this.recording) this.addFrameToGIF()    
    }.bind(this)
    animate() 
}

LoadRobotRecord.prototype._start = function(){
    this.action.paused = false; 
    this.recording = true;
}

LoadRobotRecord.prototype.toGIF = function(data){
    // Record animation as GIF
    this.canvas = data.canvas
    this.delay_between_frames = data.delay_between_frames || 0.1
    this.gif = new GIF({
        workers: data.workers,
        width: this.canvas.width,
        height: this.canvas.height,
        quality: data.quality,
        workerScript: data.workerScript || "gif.worker.js"
    })
    let auto = data.auto || false
    let finished = false    

    this.gif.on('finished', function(blob) {
        if(!finished) {
            finished = true;
            window.open(URL.createObjectURL(blob));
        } 
    });

    this.gif.on('progress', function(p) { 
        gifControl.Converting = p * 100; 
        
    });

    this.mixer.addEventListener( 'loop', function( e ) { 
        this.recording = false;  
        this.action.paused = true;
        this.gif.render();
    }.bind(this));

    var gifControl = {
        StartRecord : function() {
            this._start()
            this.action.reset();
            //this.gif.abort()  %TODO not working to reset gif
        }.bind(this),
        Converting : 0
    }

    Object.defineProperty(gifControl, 'Recording', {
        get: function() {
            return this.action !== null ? this.action.time : 0;
        }.bind(this)
    });
    
    if (auto) animate() 
    else this.gui.add(gifControl, 'StartRecord')
    this.gui.add(gifControl, 'Recording', 0, this.track.duration).listen();
    this.gui.add(gifControl, 'Converting', 0, 100).listen();

}

LoadRobotRecord.prototype.addFrameToGIF = function(data) {
    this.gif.addFrame(this.canvas, {delay: this.delay_between_frames, copy: true});
}

LoadRobotRecord.prototype.play = function(data) {
    // Play animation in loop
    let control_gui = data.control_gui || false 
    if (control_gui) clipControl( this.gui, this.action, this.mixer, this.track, this.t_f, this.t_d );
    this._start()

}


function  clipControl( gui, action, mixer, track, t_f, t_d) {
    
    var folder = gui.addFolder( "Clip '" + track.name + "'" ),
        API = {
            'play()': function() { action.play() },
            'stop()': function() { action.stop() },
            'reset()': function() { action.reset() },
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
}; 

