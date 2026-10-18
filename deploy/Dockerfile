FROM node:24-bookworm-slim@sha256:0e0ff40c39bc087845bfb27465a0df4ea419520094bc35842ff83dd8cbe6f9b6 AS build
WORKDIR /app
COPY package.json package-lock.json ./
COPY .npmrc ./
COPY adapters/vendor/package.json adapters/vendor/package.json
COPY apps/mcp/package.json apps/mcp/package.json
COPY apps/web/package.json apps/web/package.json
COPY packages/http-contracts/package.json packages/http-contracts/package.json
RUN npm ci --no-audit --no-fund
COPY . .
RUN npm run build && test -s dist/web/index.html && test -f adapters/vendor/src/service-main.ts

FROM node:24-bookworm-slim@sha256:0e0ff40c39bc087845bfb27465a0df4ea419520094bc35842ff83dd8cbe6f9b6 AS production-dependencies
WORKDIR /app
COPY package.json package-lock.json ./
COPY .npmrc ./
COPY adapters/vendor/package.json adapters/vendor/package.json
COPY apps/mcp/package.json apps/mcp/package.json
COPY apps/web/package.json apps/web/package.json
COPY packages/http-contracts/package.json packages/http-contracts/package.json
RUN npm ci --omit=dev --no-audit --no-fund && npm cache clean --force

FROM node:24-bookworm-slim@sha256:0e0ff40c39bc087845bfb27465a0df4ea419520094bc35842ff83dd8cbe6f9b6 AS runtime
ENV NODE_ENV=production \
    ROBOCLAW_HOST=0.0.0.0 \
    ROBOCLAW_PORT=57000 \
    ROBOCLAW_STATE_DIR=/data
WORKDIR /app
COPY --from=production-dependencies /app/node_modules ./node_modules
COPY --from=production-dependencies /app/package*.json ./
COPY --from=build /app/adapters/vendor ./adapters/vendor
COPY --from=build /app/apps/mcp ./apps/mcp
COPY --from=build /app/apps/web/package.json ./apps/web/package.json
COPY --from=build /app/packages ./packages
COPY --from=build /app/dist/web ./dist/web
COPY THIRD_PARTY_NOTICES ./THIRD_PARTY_NOTICES
# Host-created source files can be 0600; public code/assets must be readable by
# the non-root runtime. Never broaden permissions on private /data state.
RUN chmod -R a+rX /app/adapters /app/apps /app/packages /app/dist/web \
    && mkdir /data && chown node:node /data && chmod 700 /data
USER node
EXPOSE 57000
HEALTHCHECK --interval=30s --timeout=5s --start-period=20s --retries=3 \
    CMD ["node", "-e", "const req=require('node:http').get('http://127.0.0.1:57000/healthz',{headers:{host:new URL(process.env.ROBOCLAW_PUBLIC_ORIGIN||'http://127.0.0.1:57000').host}},r=>{r.resume();process.exitCode=r.statusCode===200?0:1});req.setTimeout(4000,()=>req.destroy());req.on('error',()=>{process.exitCode=1})"]
CMD ["node", "adapters/vendor/src/service-main.ts"]
