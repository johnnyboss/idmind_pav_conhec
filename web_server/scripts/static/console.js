

function map_update_img(url) {
    if (url != this.current_img_url) {
        this.map_img = new Image();
        this.map_img.src = url;
        var obj = this;
        this.map_img.onload = function() {obj.draw_img();}
        this.current_img_url = url;
    }
}

function map_draw_img() {
    // if (!this.map_img.complete)
    //     return;
    var w = this.map_img.width;
    var h = this.map_img.height;
    
    this.canv_bg[0].width  = w;
    this.canv_bg[0].height = h;
    this.canv_fg[0].width  = w;
    this.canv_fg[0].height = h;
    this.canv_rb[0].width  = w;
    this.canv_rb[0].height = h;

    var ctx = this.canv_bg[0].getContext('2d');
    ctx.drawImage(this.map_img, 0, 0, w, h);
}

function map_draw_robot(x, y, t, r) {
    var ctx = this.ctx_fg;
    ctx.beginPath();
    ctx.fillStyle = "#00EE00";
    ctx.moveTo(x, y);
    ctx.arc(x, y, r, 0.0, 2 * Math.PI, true);
    ctx.fill();
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.strokeStyle = "#000000";
    ctx.moveTo(x, y);
    ctx.lineTo(x + r*Math.cos(t), y - r*Math.sin(t));
    ctx.stroke();
}

function map_draw_lidar(x, y) {
}

function map_draw_particle(x, y, t) {
    var ctx = this.ctx_fg;
}

function map_redraw() {
    this.ctx_fg = this.canv_fg[0].getContext('2d');
    this.ctx_fg.clearRect(0, 0, this.canv_fg[0].width, this.canv_fg[0].height);
}

function map_mousedown(event) {
    var obj = event.data.obj;
    event.preventDefault();
    var x = (event.pageX - $(this).offset().left);
    var y = (event.pageY - $(this).offset().top);
    obj.mouseDownX = x;
    obj.mouseDownY = y;
    obj.mouseX = x;
    obj.mouseY = y;
    obj.clicked = 1;
    map.draw_mouse();
}

function map_mouseup(event) {
    var obj = event.data.obj;
    if (obj.clicked>0) {
        obj.mouseX = (event.pageX - $(this).offset().left);
        obj.mouseY = (event.pageY - $(this).offset().top);

        var x = obj.mouseDownX;
        var y = obj.mouseDownY;
        var t = Math.atan2(obj.mouseY-y, obj.mouseX-x);

        if (event.ctrlKey)
            set_dct_goal(x, y, t);
        else if (event.shiftKey)
            set_location(x, y, t);
        else
            set_route_goal(x, y, t);
    }
    obj.clicked = 0;
    map.draw_mouse();  // clears mouse since clicked=0
}

function map_mousemove(event) {
    var obj = event.data.obj;
    if (obj.clicked>0) {
        obj.mouseX = (event.pageX - $(this).offset().left);
        obj.mouseY = (event.pageY - $(this).offset().top);
        map.draw_mouse();
    }
}

function map_draw_mouse() {
    if (this.canv_rb.is(':visible')) {
        var ctx = this.canv_rb[0].getContext('2d');
        ctx.clearRect(0, 0, this.canv_rb[0].width, this.canv_rb[0].height);
        if (this.clicked==1) {
            var x1 = this.mouseDownX;
            var y1 = this.mouseDownY;
            var x2 = this.mouseX;
            var y2 = this.mouseY;
            ctx.lineWidth = 1.5;
            ctx.beginPath();
            ctx.strokeStyle = "#DDAA00";
            ctx.arc(x1, y1, 5, 0.0, 2 * Math.PI, true);
            ctx.stroke();
            ctx.beginPath();
            ctx.strokeStyle = "#DDDD00";
            ctx.moveTo(x1, y1);
            ctx.lineTo(x2, y2);
            ctx.stroke();
        }
    }
}

function map_ping(rtt) {
    // console.log("map_ping "+this.rot_idx);
    var chars = ['-', '\\', '|', '/'];
    $('#net').html(chars[this.rot_idx] + " (" + rtt + " ms)");
    this.rot_idx = (this.rot_idx+1)%chars.length;
}

function Map() {
    // methods
    this.update_img = map_update_img;
    this.draw_img = map_draw_img;
    this.draw_robot = map_draw_robot;
    this.draw_lidar = map_draw_lidar;
    this.draw_particle = map_draw_particle;
    this.redraw = map_redraw;
    this.draw_mouse = map_draw_mouse;
    this.ping = map_ping;
    
    // properties
    this.current_img_url = "";
    this.current_map = "";
    this.clicked = 0;
    this.canv_bg = $("#map_canvas_bg");
    this.canv_fg = $("#map_canvas_fg");
    this.canv_rb = $("#map_canvas_rb");
    this.rot_idx = 0;

    this.canv_rb.bind('mousedown', {'obj':this}, map_mousedown);
    this.canv_rb.bind('mousemove', {'obj':this}, map_mousemove);
    this.canv_rb.bind('mouseup', {'obj':this}, map_mouseup);
}

function set_status(text) {
    $('#status').html(text);
}





function refresh_map() {
    // console.log("refreshing_map");
    var d = new Date();
    $.get('/do/map', update_map);
    last_ping = d.getTime();
}

function change_map() {
    var sel = $('#map_select')[0];
    var idx = sel.selectedIndex;
    var mid = sel[idx].getAttribute('value');
    sel.disabled = true;
    map.current_map = mid;
    console.log('trying to change map to '+mid);
}

function update_map(data) {
    // console.log("updating map");
    var d = new Date();
    var img;
    if ($("#map_canvas_bg").is(':visible') &&
        $("#map_canvas_fg").is(':visible')) {
        // initialize map
        map.redraw();
        // draw map info
        $(data).find('map').each(function() {
            var ctx = map.ctx_fg;
            // handle background map
            img = $(this).find('img').text();
            map.update_img(img);
            // handle robot
            rob = $(this).find('robot');
            var x = parseFloat(rob.attr('x'));
            var y = parseFloat(rob.attr('y'));
            var t = parseFloat(rob.attr('t'));
            var r = parseFloat(rob.attr('r'));
            map.draw_robot(x, y, t, r);
            // handle LIDAR scan
            ctx.fillStyle = "#FF0000";
            lidar = $(this).find('lidar');
            lidar.find('p').each(function() {
                ctx.fillRect($(this).attr('x'), $(this).attr('y'), 1, 1);
            });
	    // handle GOAL
            ctx.fillStyle = "#FF0000";
            lidar = $(this).find('goal_point');
            lidar.find('p').each(function() {
                ctx.lineWidth = 1.5;
                ctx.beginPath();
                ctx.fillStyle = "#FF0000";
                x1 = $(this).attr('x');
		y1 = $(this).attr('y')
                ctx.arc(x1, y1, 6, 0.0, 2 * Math.PI, true);
		ctx.fill();
                ctx.stroke();
            });
	    //handle NAV_POINTS
	    ctx.fillStyle = "#FF0000";
	    lidar = $(this).find('nav_points');
	    lidar.find('p').each(function() {
		ctx.lineWidth = 1.5;
                ctx.beginPath();
                ctx.strokeStyle = "#0000FF";
                x1 = $(this).attr('x');
		y1 = $(this).attr('y')
                ctx.arc(x1, y1, 5, 0.0, 2 * Math.PI, true);
                ctx.stroke();
            });

            // handle particle cloud
            ctx.fillStyle = "#0000FF";
            lidar = $(this).find('particlecloud');
            lidar.find('p').each(function() {
                ctx.fillRect($(this).attr('x'), $(this).attr('y'), 1, 1);
            });
            // handle battery
            batt = $(this).find('battery').text();
            if (batt) {
                $('#battery').html(batt);
            }
	    // handle position
            pos = $(this).find('position').text();
            if (pos) {
                $('#position').html(pos);
            }
            // handle map menu
            var maps = $(this).find('maps');
            var cur  = maps.attr('current');
            if (map.current_map != cur) {
                var html = '<select id="map_select" onchange="change_map()">';
                maps.find('m').each(function() {
                    var mid = $(this).attr('id');
                    if (cur == mid) {
                        html += '<option selected value="'+mid+'">'+$(this).text()+'</option>';
                    } else {
                        html += '<option value="'+mid+'">'+$(this).text()+'</option>';
                    }
                });
                $('#map_menu').html(html);
                map.current_map = cur;
            }
        });
        // draw moving thingy
        map.ping(d.getTime()-last_ping);
        // draw mouse interaction
        map.draw_mouse();
    }
    setTimeout("refresh_map()", 500);
}

function set_location(x, y, t) {
    $.get('/do/set_location?'+x+'&'+y+'&'+t, set_response);
}

function set_dct_goal(x, y, t) {
    $.get('/do/set_dct_goal?'+x+'&'+y+'&'+t, set_response);
}

function set_route_goal(x, y, t) {
    $.get('/do/set_route_goal?'+x+'&'+y+'&'+t, set_response);
}

function set_response(data) {
    set_status(data);
}



var map;


var last_ping;

function main() {
    map = new Map();
    refresh_map();
}

// Main entry
$(document).ready(main);

// EOF
