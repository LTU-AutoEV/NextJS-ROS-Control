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

    // Import these filetypes with file loader
    // Things like LeafLet require this to not throw errors
    config.module.rules.push(
        {test: /\.(gif|svg|jpg|png)$/, use: ['file-loader']})

    return config
  }
})
