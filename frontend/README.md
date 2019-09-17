# Frontend

- [Frontend](#frontend)
  - [Summary](#summary)
  - [Environment Variables](#environment-variables)
  - [Changelog](#changelog)
  - [Project commands](#project-commands)
    - [Setup the project](#setup-the-project)
    - [Compiles and hot-reloads for development](#compiles-and-hot-reloads-for-development)
    - [Compiles and minifies for production](#compiles-and-minifies-for-production)
    - [Generates the frontend documentation](#generates-the-frontend-documentation)
    - [Lints and fixes files](#lints-and-fixes-files)
    - [Run your tests](#run-your-tests)
    - [Run your end-to-end tests](#run-your-end-to-end-tests)
    - [Run your unit tests](#run-your-unit-tests)

## Summary

The frontend is been develop using the Vue.js framework with bootstrap toolbox and so is web base.
The interface is there to offer an easy way for operators to connect to any robot in services and perform some specific actions on them.

> We currently do not support mobile devices.

## Environment Variables
The UI, through Vue/webpack, uses environment variables to set come variables used during execution. The variables are set inside a [.env file](https://cli.vuejs.org/guide/mode-and-env.html#modes).

An example of this is shown inside .env.example. You would need to either create a new file in the same format or create a copy of the file and change its values to the ones you desired.s

- VUE_APP_SERVER_URL: The URL of the server to connect to.
- VUE_APP_SERVER_ROOM_NAME: The room name to connect to on the server.

## Changelog
The changelog file shows changes between versions since v0.1.0. It was created on September 17th 2019, before that the only changelogs available for the UI are the commits done on the repo.

## Project commands

### Setup the project
```sh
npm install
```

### Compiles and hot-reloads for development
```sh
npm run serve
```

### Compiles and minifies for production
```sh
npm run build
```

### Generates the frontend documentation
```sh
npm run docs
```

### Lints and fixes files
> ESLint is use for linting the code.
```sh
npm run lint
```

### Run your tests
> There is currently no test suits even though the packages (Mocha + Nightwatch) are included in dependcies.
```sh
npm run test
```

### Run your end-to-end tests
> Only runs the e2e (Nightwatch) test suits.
```sh
npm run test:e2e
```

### Run your unit tests
> Only runs the unit (mocha) test suits.
```sh
npm run test:unit
```
