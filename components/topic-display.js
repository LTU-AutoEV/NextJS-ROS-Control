import React, { Component } from 'react';
import Link from 'next/link'

class TopicDisplay extends Component {
  render() {
    return (
      <div>
        <span className="font-weight-bold">{this.props.name}</span>
        &nbsp; &nbsp;
        <span className="font-italic">{this.state.data}</span>
      </div>
    )
  }

  constructor(props) {
    super(props)

    // Import some stuff client-only
    if(typeof window !== 'undefined' && window.document && window.document.createElement) {
      this.roslib = require('roslib')
      this.ros = require('clientOnly/ros.js')
    }

    this.state = {
      data: 0
    }
  }

  componentDidMount() {
    var that = this;
    this.sub = new this.roslib.Topic({
      ros: this.ros,
      name: that.props.topic,
      messageType: that.props.type
    })

    var that = this;
    this.sub.subscribe(function(msg) {
      that.setState({data: msg.data});
    })
  }

  componentWillUnmount() {
    this.sub.unsubscribe()
    this.sub = null
  }
}

export default TopicDisplay
