var math = require('generic_modules/math');
var noise = require('perlin');
var Quadtree = require('quadtree');
var seedrandom = require('seedrandom');
var _ = require('lodash');

var defaultConfig = {
    HIGHWAY_SEGMENT_WIDTH: 80,
    DEFAULT_SEGMENT_WIDTH: 40,
    DEFAULT_SEGMENT_LENGTH: 500,
    HIGHWAY_SEGMENT_LENGTH: 500,
    MINIMUM_INTERSECTION_DEVIATION: 30,
    ROAD_SNAP_DISTANCE: 200,
    RANDOM_STRAIGHT_ANGLE: function() { return (Math.random() - 0.5) * 2 * 10; },
    RANDOM_BRANCH_ANGLE: function() { return (Math.random() - 0.5) * 2 * 20; },
    HIGHWAY_BRANCH_POPULATION_THRESHOLD: 0.3,
    NORMAL_BRANCH_POPULATION_THRESHOLD: 0.1,
    HIGHWAY_BRANCH_PROBABILITY: 0.1,
    DEFAULT_BRANCH_PROBABILITY: 0.4,
    NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY: 10,
    SEGMENT_COUNT_LIMIT: 2000,
    QUADTREE_BOUNDS: { x: -15000, y: -15000, width: 30000, height: 30000 },
    QUADTREE_MAX_OBJECTS: 10,
    QUADTREE_MAX_LEVELS: 10,
    MIN_SPEED_PROPORTION: 0.1
};

function minDegreeDifference(a, b) {
    var diff = Math.abs(a - b);
    return Math.min(diff, 360 - diff);
}

var segmentCounter = 0;
function Segment(start, end, t, q) {
    var self = this;
    this.id = segmentCounter++;
    this.r = { start: { x: start.x, y: start.y }, end: { x: end.x, y: end.y } };
    this.t = t || 0;
    this.q = q || {};
    this.links = { b: [], f: [] };
    this.width = this.q.highway ? defaultConfig.HIGHWAY_SEGMENT_WIDTH : defaultConfig.DEFAULT_SEGMENT_WIDTH;
    this.roadRevision = 0;
    this.dirRevision = undefined;
    this.lengthRevision = undefined;
    this.cachedDir = undefined;
    this.cachedLength = undefined;
    this.r.setStart = function(val) { self.r.start = val; self.roadRevision++; };
    this.r.setEnd = function(val) { self.r.end = val; self.roadRevision++; };
    this.users = [];
    var speedAndCapacity = this.q.highway ? [1200, 12] : [800, 6];
    this.maxSpeed = speedAndCapacity[0];
    this.capacity = speedAndCapacity[1];
}
Segment.prototype.dir = function() {
    if (this.dirRevision != this.roadRevision) {
        this.dirRevision = this.roadRevision;
        var vector = math.subtractPoints(this.r.end, this.r.start);
        this.cachedDir = -1 * math.sign(math.crossProduct({x:0, y: 1}, vector)) * math.angleBetween({x: 0, y: 1}, vector);
    }
    return this.cachedDir;
};
Segment.prototype.length = function() {
    if (this.lengthRevision != this.roadRevision) {
        this.lengthRevision = this.roadRevision;
        this.cachedLength = math.length(this.r.start, this.r.end);
    }
    return this.cachedLength;
};
Segment.prototype.split = function(point, segment, segmentList, qTree) {
    var startIsBackwards = this.startIsBackwards();
    var splitPart = segmentFactory.fromExisting(this);
    addSegment(splitPart, segmentList, qTree);
    splitPart.r.setEnd(point);
    this.r.setStart(point);
    splitPart.links.b = this.links.b.slice(0);
    splitPart.links.f = this.links.f.slice(0);
    var firstSplit, secondSplit, fixLinks;
    if (startIsBackwards) {
      firstSplit = splitPart;
      secondSplit = this;
      fixLinks = splitPart.links.b;
    } else {
      firstSplit = this;
      secondSplit = splitPart;
      fixLinks = splitPart.links.f;
    }
    var self = this;
    _.each(fixLinks, function(link) {
      var index = link.links.b.indexOf(self);
      if (index != -1) {
        link.links.b[index] = splitPart;
      } else {
        index = link.links.f.indexOf(self);
        if (index != -1) {
            link.links.f[index] = splitPart;
        }
      }
    });
    firstSplit.links.f = [segment, secondSplit];
    secondSplit.links.b = [segment, firstSplit];
    segment.links.f.push(firstSplit, secondSplit);
};
Segment.prototype.startIsBackwards = function() {
    if (this.links.b.length > 0) {
      return math.equalV(this.links.b[0].r.start, this.r.start) ||
             math.equalV(this.links.b[0].r.end, this.r.start);
    } else if (this.links.f.length > 0) {
      return math.equalV(this.links.f[0].r.start, this.r.end) ||
             math.equalV(this.links.f[0].r.end, this.r.end);
    }
    return false;
};
Segment.prototype.linksForEndContaining = function(segment) {
    if (this.links.b.indexOf(segment) > -1) return this.links.b;
    if (this.links.f.indexOf(segment) > -1) return this.links.f;
    return undefined;
};
Segment.prototype.currentSpeed = function() {
    return Math.min(defaultConfig.MIN_SPEED_PROPORTION, 1 - Math.max(0, this.users.length - 1) / this.capacity) * this.maxSpeed;
};
Segment.prototype.cost = function() {
    return this.length() / this.currentSpeed();
};
Segment.prototype.costTo = function(other, fromFraction) {
    var segmentEnd = this.endContaining(other);
    var cost = this.cost();
    if (fromFraction !== undefined) {
        if (segmentEnd === "start") {
            return cost * fromFraction;
        } else {
            return cost * (1 - fromFraction);
        }
    }
    return cost * 0.5;
};
Segment.prototype.endContaining = function(segment) {
    var startBackwards = this.startIsBackwards();
    if (this.links.b.indexOf(segment) != -1) {
        return startBackwards ? "start" : "end";
    } else if (this.links.f.indexOf(segment) != -1) {
        return startBackwards ? "end" : "start";
    }
    return undefined;
};
Segment.prototype.neighbours = function() {
    return this.links.f.concat(this.links.b);
};


var segmentFactory = {
    fromExisting: function(segment) {
        return new Segment(segment.r.start, segment.r.end, segment.t, _.cloneDeep(segment.q));
    },
    usingDirection: function(start, dir, length, t, q) {
        dir = dir === undefined ? 90 : dir;
        length = length || defaultConfig.DEFAULT_SEGMENT_LENGTH;
        var end = {
            x: start.x + length * math.sinDegrees(dir),
            y: start.y + length * math.cosDegrees(dir)
        };
        return new Segment(start, end, t, q);
    }
};

function doRoadSegmentsIntersect(r1, r2) {
  return math.doLineSegmentsIntersect(r1.start, r1.end, r2.start, r2.end, true);
}

function addSegment(segment, segmentList, qTree) {
  segmentList.push(segment);
  var limits = {
      x: Math.min(segment.r.start.x, segment.r.end.x) - segment.width,
      y: Math.min(segment.r.start.y, segment.r.end.y) - segment.width,
      width: Math.abs(segment.r.start.x - segment.r.end.x) + 2 * segment.width,
      height: Math.abs(segment.r.start.y - segment.r.end.y) + 2 * segment.width,
  };
  qTree.insert(Object.assign({}, limits, { o: segment }));
}

function localConstraints(segment, segments, qTree, config, debugData) {
    var action = { priority: 0, func: undefined, q: {} };
    var matches = qTree.retrieve({
        x: Math.min(segment.r.start.x, segment.r.end.x) - segment.width,
        y: Math.min(segment.r.start.y, segment.r.end.y) - segment.width,
        width: Math.abs(segment.r.start.x - segment.r.end.x) + 2 * segment.width,
        height: Math.abs(segment.r.start.y - segment.r.end.y) + 2 * segment.width,
    });

    matches.forEach(function(match) {
        var other = match.o;
        if (action.priority <= 4) {
            var intersection = doRoadSegmentsIntersect(segment.r, other.r);
            if (intersection) {
                if (!action.q.t || intersection.t < action.q.t) {
                    action.q.t = intersection.t;
                    var capturedOther = other;
                    var capturedIntersection = intersection;
                    action.priority = 4;
                    action.func = function() {
                        if (minDegreeDifference(capturedOther.dir(), segment.dir()) < config.MINIMUM_INTERSECTION_DEVIATION) return false;
                        capturedOther.split(capturedIntersection, segment, segments, qTree);
                        segment.r.setEnd(capturedIntersection);
                        segment.q.severed = true;
                        if (debugData) debugData.intersections.push(capturedIntersection);
                        return true;
                    };
                }
            }
        }
        if (action.priority <= 3) {
            if (math.length(segment.r.end, other.r.end) <= config.ROAD_SNAP_DISTANCE) {
                var capturedOther2 = other;
                action.priority = 3;
                action.func = function() {
                    segment.r.setEnd(capturedOther2.r.end);
                    segment.q.severed = true;
                    if (debugData) debugData.snaps.push(capturedOther2.r.end);
                    return true;
                }
            }
        }
        if (action.priority <= 2) {
            var dist = math.distanceToLine(segment.r.end, other.r.start, other.r.end);
            if (dist.distance2 < config.ROAD_SNAP_DISTANCE * config.ROAD_SNAP_DISTANCE && dist.lineProj2 >= 0 && dist.lineProj2 <= dist.length2) {
                var capturedOther3 = other;
                var capturedPoint = dist.pointOnLine;
                action.priority = 2;
                action.func = function() {
                    if (minDegreeDifference(capturedOther3.dir(), segment.dir()) < config.MINIMUM_INTERSECTION_DEVIATION) return false;
                    segment.r.setEnd(capturedPoint);
                    segment.q.severed = true;
                    capturedOther3.split(capturedPoint, segment, segments, qTree);
                    if (debugData) debugData.intersectionsRadius.push(capturedPoint);
                    return true;
                }
            }
        }
    });

    if (action.func) return action.func();
    return true;
}

var globalGoals = (function() {
  return {
    generate: function(previousSegment, config, heatmap) {
      var newBranches = [];
      if (!previousSegment.q.severed) {
        var template = function(direction, length, t, q) {
          return segmentFactory.usingDirection(previousSegment.r.end, direction, length, t, q);
        };
        var templateContinue = _.partialRight(template, previousSegment.length(), 0, previousSegment.q);
        var templateBranch = _.partialRight(template, config.DEFAULT_SEGMENT_LENGTH, previousSegment.q.highway ? config.NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY : 0);
        var continueStraight = templateContinue(previousSegment.dir());
        var straightPop = heatmap.popOnRoad(continueStraight.r);

        if (previousSegment.q.highway) {
          var randomStraight = templateContinue(previousSegment.dir() + config.RANDOM_STRAIGHT_ANGLE());
          var randomPop = heatmap.popOnRoad(randomStraight.r);
          var roadPop;
          if (randomPop > straightPop) {
            newBranches.push(randomStraight);
            roadPop = randomPop;
          } else {
            newBranches.push(continueStraight);
            roadPop = straightPop;
          }
          if (roadPop > config.HIGHWAY_BRANCH_POPULATION_THRESHOLD) {
            if (Math.random() < config.HIGHWAY_BRANCH_PROBABILITY) {
              newBranches.push(templateContinue(previousSegment.dir() - 90 + config.RANDOM_BRANCH_ANGLE()));
            } else if (Math.random() < config.HIGHWAY_BRANCH_PROBABILITY) {
              newBranches.push(templateContinue(previousSegment.dir() + 90 + config.RANDOM_BRANCH_ANGLE()));
            }
          }
        } else if (straightPop > config.NORMAL_BRANCH_POPULATION_THRESHOLD) {
          newBranches.push(continueStraight);
        }
        if (straightPop > config.NORMAL_BRANCH_POPULATION_THRESHOLD) {
          if (Math.random() < config.DEFAULT_BRANCH_PROBABILITY) {
            newBranches.push(templateBranch(previousSegment.dir() - 90 + config.RANDOM_BRANCH_ANGLE()));
          } else if (Math.random() < config.DEFAULT_BRANCH_PROBABILITY) {
            newBranches.push(templateBranch(previousSegment.dir() + 90 + config.RANDOM_BRANCH_ANGLE()));
          }
        }
      }
      newBranches.forEach(function(branch) {
          branch.setupBranchLinks = function() {
              _.each(previousSegment.links.f, function(link) {
                  branch.links.b.push(link);
                  var links = link.linksForEndContaining(previousSegment);
                  if (links) {
                    links.push(branch);
                  }
              }, branch);
              previousSegment.links.f.push(branch);
              branch.links.b.push(previousSegment);
          };
      });
      return newBranches;
    }
  };
})();

function generate(seed, options) {
    var config = Object.assign({}, defaultConfig, options);
    Math.random = seedrandom(seed);
    noise.seed(Math.random());

    var segments = [];
    var priorityQ = [];
    var qTree = new Quadtree(config.QUADTREE_BOUNDS, config.QUADTREE_MAX_OBJECTS, config.QUADTREE_MAX_LEVELS);
    var debugData = { intersections: [], snaps: [], intersectionsRadius: [] };
    var heatmap = {
        populationAt: function(x, y) {
            var v1 = (noise.simplex2(x/10000, y/10000) + 1) / 2;
            var v2 = (noise.simplex2(x/20000 + 500, y/20000 + 500) + 1) / 2;
            var v3 = (noise.simplex2(x/20000 + 1000, y/20000 + 1000) + 1) / 2;
            return Math.pow((v1 * v2 + v3) / 2, 2);
        },
        popOnRoad: function(r) {
            return (this.populationAt(r.start.x, r.start.y) + this.populationAt(r.end.x, r.end.y))/2;
        }
    };

    var rootSegment = new Segment({x: 0, y: 0}, {x: config.HIGHWAY_SEGMENT_LENGTH, y: 0}, 0, {highway: true});
    var oppositeDirection = segmentFactory.fromExisting(rootSegment);
    var newEnd = { x: rootSegment.r.start.x - config.HIGHWAY_SEGMENT_LENGTH, y: oppositeDirection.r.end.y };
    oppositeDirection.r.setEnd(newEnd);
    oppositeDirection.links.b.push(rootSegment);
    rootSegment.links.b.push(oppositeDirection);
    priorityQ.push(rootSegment, oppositeDirection);

    while (priorityQ.length > 0 && segments.length < config.SEGMENT_COUNT_LIMIT) {
        priorityQ.sort(function(a, b) { return a.t - b.t; });
        var minSegment = priorityQ.shift();

        var accepted = localConstraints(minSegment, segments, qTree, config, debugData);
        if (accepted) {
            if (minSegment.setupBranchLinks) minSegment.setupBranchLinks();
            addSegment(minSegment, segments, qTree);
            var newBranches = globalGoals.generate(minSegment, config, heatmap);
            newBranches.forEach(function(branch) {
                branch.t = minSegment.t + 1 + branch.t;
                priorityQ.push(branch);
            });
        }
    }
    return { segments: segments, qTree: qTree, heatmap: heatmap, debugData: debugData };
}

module.exports = { generate: generate, Segment: Segment };
