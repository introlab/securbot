
module.exports = {
  plugins: [],
  recurseDepth: 10,
  source: {
    include: ['main.js', 'ipcInit.js', 'renderer.js'],
    exclude: [],
    includePattern: '.+\\.js(doc|x)?$',
    excludePattern: '(^|\\/|\\\\)_',
  },
  sourceType: 'module',
  opts: {
    template: 'node_modules/minami',
    destination: '../../docs/ros/JSDoc',
    recurse: false,
    readme: 'README.md',
  },
  tags: {
    allowUnknownTags: true,
    dictionaries: ['jsdoc', 'closure'],
  },
  templates: {
    cleverLinks: false,
    monospaceLinks: true,
    useLongnameInNav: false,
    showInheritedInNav: true,
  },
};
