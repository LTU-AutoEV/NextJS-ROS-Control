import React, { Component } from 'react';
import * as ReactLeaflet from 'react-leaflet-universal'
import dynamic from 'next/dynamic'
const { Map: LeafletMap, TileLayer, Marker, Popup, ZoomControl, ScaleControl, LatLng} = ReactLeaflet

export default class InteractiveMap extends Component {
  constructor() {
    super()

    // Import some stuff client-only
    if(typeof window !== 'undefined' && window.document && window.document.createElement) {
      console.log('clientOnly');
      // Load leaflet css for this page
      import('leaflet/dist/leaflet.css')

      // Ensure leaflet global L is loaded
      require('leaflet')
      require('leaflet.offline')

      // Ensure ros connection is started and availaible
      this.roslib = require('roslib')
      this.ros = require('clientOnly/ros.js')
    }

    this.state = {
      lat: 42.475,
      lng: -83.249,
      zoom: 15,
      waypoints: [],
      markers: [],
      currentPosMarker: undefined, // Vehicle Location
      currentWpMarker: undefined   // Current WP Location
    }
  }

  componentDidMount() {
    var that = this

    this.gpsSub = new this.roslib.Topic({
      ros: this.ros,
      name: '/fix',
      messageType: 'sensor_msgs/NavSatFix'
    })

    this.waypointSub = new this.roslib.Topic({
      ros: this.ros,
      name: '/mapserver/add_waypoint',
      messageType: 'geometry_msgs/Point'
    })

    this.gpsSub.subscribe(function(msg) {
      let {currentPosMarker} = that.state;
      const position = new L.LatLng(msg.latitude, msg.longitude);

      // Update position of marker and re-center view
      currentPosMarker.setLatLng(position);
      that.updateView();
      that.setState({lat: msg.latitude, lng: msg.longitude, currentPosMarker})
    })

    this.waypointSub.subscribe(function(msg) {
      const position = new L.LatLng(msg.x, msg.y);
      let {currentWpMarker} = that.state;

      if (msg.z > 0) {
        // Current waypoint marker
        currentWpMarker.setLatLng(position);
        that.setState({currentWpMarker});
      } else {
        // All others
        const {markers, waypoints} = that.state;

        // if all zeros, clear waypoints
        if (msg.x == 0.0 && msg.y == 0.0) {
          // Remove all markers from tha map
          for(var i = 0; i < markers.length; i++){
            that.map.removeLayer(markers[i]);
          }
          // 'clear' the markers
          markers.length = 0;
          waypoints.length = 0;

          //Also clear current waypoint
          currentWpMarker.setLatLng(new L.LatLng(0,0));
        } else {
          // Add marker to map
          var marker = L.circleMarker(position, {color: '#000', weight: 1});
          marker.setRadius(4);
          marker.addTo(that.map);
          // Keep a ref to the marker
          markers.push(marker);
          waypoints.push(position);
        }

        that.updateView();
        that.setState({markers, waypoints, currentWpMarker})
      }
    })

    this.map = L.map('map');
    let baseLayer =  L.tileLayer.offline('http://{s}.tile.osm.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(this.map);

    // Create current WP it for the first time
    const position = new L.LatLng(this.state.lat, this.state.lng);
    let {currentWpMarker, currentPosMarker} = this.state;
    currentWpMarker = L.circleMarker(position, {color: 'green', weight: 0, fillOpacity: 0.4});
    currentWpMarker.setRadius(10);
    currentWpMarker.addTo(this.map);

    // Create current pos marker
    currentPosMarker = L.circleMarker(position);
    currentPosMarker.setRadius(10);
    currentPosMarker.addTo(this.map);

    // add buttons to save tiles in area viewed
    let control = L.control.savetiles(baseLayer, {
      'confirm': function(layer, succescallback) {
        if (window.confirm("Save " + layer._tilesforSave.length)) {
          succescallback();
        }
      },
      'confirmRemoval': function(layer, successCallback) {
        if (window.confirm("Remove all the tiles?")) {
          successCallback();
        }
      },
      'saveText': '&#x1f4be;', //floppy
      'rmText': '&#x1F5D1' //wastebasket
    });

    control.addTo(this.map);

    baseLayer.on('storagesize', function(e) {
      document.getElementById('storage').innerHTML = e.storagesize;
    })

    //events while saving a tile layer
    let progress = 0;
    baseLayer.on('savestart', function(e) {
      progress = 0;
      document.getElementById("total").innerHTML = e._tilesforSave.length;
    });

    baseLayer.on('savetileend', function(e) {
      progress++;
      let s = document.getElementById('storage').innerHTML;
      let out = '';
      if (s.includes(')')) {
        s = s.split(')')[1];
      }

      document.getElementById('storage').innerHTML = '(' + progress + ') ' + s;

    });

    baseLayer.on('loadend', function(e) {
      alert("Saved all tiles");
    });

    baseLayer.on('tilesremoved', function(e) {
      alert("Removed all tiles");
    });

    let followView = false;
    let followZoom = 15;

    this.map.setView(position, this.state.zoom);
    this.setState({currentWpMarker, currentPosMarker});
  }

  componentWillUnmount() {
    this.gpsSub.unsubscribe()
    this.gpsSub = null
    this.waypointSub.unsubscribe()
    this.waypointSub = null
  }

  updateView() {
    // Set the map view around the points including the location of the car
    //let wp = this.state.waypoints;
    //if (wp.length > 0) this.map.fitBounds(wp);

    // Set the map view around the current fix
    this.map.panTo([this.state.lat, this.state.lng])
  }

  render() {
    return (
      <div className='row'>
        <div id="map" className='col border rounded' style={{height:'400px'}} />
        <div id="total" />
        <div id="progress" />
        <div id="remove_tiles" />
      </div>
    );
  }
}
