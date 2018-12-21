/*
 * This file overrides the default _app.js to include the bootstrap CSS files and set some metainformation for it.
 */

import App, {Container} from 'next/app'
import React from 'react'
import Head from 'next/head'
import 'bootstrap/dist/css/bootstrap.min.css' // This includes this css file in all pages

export default class ActorHTTP extends App {
  static async getInitialProps ({ Component, router, ctx }) {
    let pageProps = {}

    if (Component.getInitialProps) {
      pageProps = await Component.getInitialProps(ctx)
    }

    return {pageProps}
  }

  render () {
    const {Component, pageProps} = this.props
    return <Container>
      <Head>
        <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no" />
      </Head>
      <Component {...pageProps} />
    </Container>
  }
}
