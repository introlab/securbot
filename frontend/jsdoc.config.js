'use strict';

module.exports = {
  "plugins": ["node_modules/jsdoc-vuejs"],
  "recurseDepth": 10,
  "source": {
    "include": ["src"],
    "exclude": ["src/assets"],
    "includePattern": ".+\\.(vue|js)?$",
    "excludePattern": "(^|\\/|\\\\)_"
  },
  "sourceType": "module",
  "opts": {
    "template": "node_modules/minami",
    "destination": "../docs/frontend/JSDoc",
    "recurse": true,
    "readme": "README.md",
  },
  "tags": {
    "allowUnknownTags": true,
    "dictionaries": ["jsdoc","closure"]
  },
  "templates": {
    "cleverLinks": false,
    "monospaceLinks": true,
    "useLongnameInNav": false,
    "showInheritedInNav": true
  }
}