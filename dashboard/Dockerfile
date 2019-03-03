# Puppetter image
FROM node:11.10.1-stretch

# Installing pdfjam and chromium dependecies
# https://github.com/GoogleChrome/puppeteer/blob/master/.ci/node8/Dockerfile.linux
RUN apt-get update && \
    apt-get -y install pdfjam xvfb gconf-service libasound2 libatk1.0-0 libc6 libcairo2 libcups2 \
      libdbus-1-3 libexpat1 libfontconfig1 libgcc1 libgconf-2-4 libgdk-pixbuf2.0-0 libglib2.0-0 \
      libgtk-3-0 libnspr4 libpango-1.0-0 libpangocairo-1.0-0 libstdc++6 libx11-6 libx11-xcb1 libxcb1 \
      libxcomposite1 libxcursor1 libxdamage1 libxext6 libxfixes3 libxi6 libxrandr2 libxrender1 libxss1 \
      libxtst6 ca-certificates fonts-liberation libappindicator1 libnss3 lsb-release xdg-utils wget && \
    rm -rf /var/lib/apt/lists/*

# Setting timezone
ENV TZ=America/Montreal
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Application directory
WORKDIR /usr/src/app

# Node install packages
COPY package.json ./
RUN npm install

# Copy all apprlication files add unwanted files .dockerignore
COPY . .

# Default application
CMD ["node", "scheduledRender.js"]
