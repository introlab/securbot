FROM node:12

ENV NODE_ENV production
ENV PORT 3000

WORKDIR /opt/web_rtc

COPY ["package.json", "package-lock.json*", "./"]

RUN npm install --production --silent

COPY . .

EXPOSE 3000

CMD npm start