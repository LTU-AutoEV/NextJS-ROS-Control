const withCSS = require('@zeit/next-css')
const { BundleAnalyzerPlugin } = require('webpack-bundle-analyzer')
const { ANALYZE } = process.env
const path = require('path')

module.exports = withCSS({
  webpack: function(config, {isServer}) {
    if (ANALYZE) {
      config.plugins.push(new BundleAnalyzerPlugin({
        analyzerMode: 'server',
        analyzerPort: isServer ? 8888 : 8889,
        openAnalyzer: true
      }))
    }

    config.resolve.alias.components = path.resolve(__dirname, 'components')
    config.resolve.alias.clientOnly = path.resolve(__dirname, 'client-only')

    // Leaflet requires different css loaders than Next.js uses
    config.module.rules.push({
      test: /leaflet\.css/,
      use: [
        'style-loader',
        'css-loader',
      ]
    });

    // To use the new css loader, we must disable the default as well
    for (var i = 0; i < config.module.rules.length; i++) {
      if (config.module.rules[i].test.test('example.css')) {
        config.module.rules[i].exclude = /leaflet\.css/
      }
    }

    // And the leaflet css tries to import some images too
    config.module.rules.push(
        {test: /\.(gif|svg|jpg|png)$/, use: ['file-loader']})

    return config
  }
})
