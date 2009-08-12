/* Copyright 2002-2009 CS Communication & Systèmes
 * Licensed to CS Communication & Systèmes (CS) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * CS licenses this file to You under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package org.orekit.data;


import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.text.ParseException;
import java.util.regex.Pattern;

import org.junit.Assert;
import org.junit.Test;
import org.orekit.errors.OrekitException;

public class NetworkCrawlerTest {

    @Test
    public void noElement() {
        File existing   = new File(url("regular-data").getPath());
        File inexistent = new File(existing.getParent(), "inexistant-directory");
        try {
            new NetworkCrawler(inexistent.toURI().toURL()).feed(new CountingLoader(".*"));
            Assert.fail("an exception should have been thrown");
        } catch (OrekitException e) {
            // expected behavior
        } catch (Exception e) {
            e.printStackTrace();
            Assert.fail("wrong exception caught");
        }
    }

    // WARNING!
    // the following test is commented out by default, as it does connect to the web
    // if you want to enable it, you will have uncomment it and to either set the proxy
    // settings according to your local network or remove the proxy authentication
    // settings if you have a transparent connection to internet
//    @Test
//    public void remote() throws java.net.MalformedURLException, OrekitException {
//
//        System.setProperty("http.proxyHost",     "proxy.your.domain.com");
//        System.setProperty("http.proxyPort",     "8080");
//        System.setProperty("http.nonProxyHosts", "localhost|*.your.domain.com");
//        java.net.Authenticator.setDefault(new AuthenticatorDialog());
//
//        CountingLoader loader = new CountingLoader(".*\\.history");
//        NetworkCrawler crawler =
//            new NetworkCrawler(new URL("http://hpiers.obspm.fr/eoppc/bul/bulc/UTC-TAI.history"));
//        crawler.setTimeout(1000);
//        crawler.feed(loader);
//        Assert.assertEquals(1, loader.getCount());
//
//    }

    @Test
    public void local() throws OrekitException {
        CountingLoader crawler = new CountingLoader(".*");
        new NetworkCrawler(url("regular-data/UTC-TAI.history"),
                           url("regular-data/de405-ephemerides/unxp0000.405"),
                           url("regular-data/de405-ephemerides/unxp0001.405"),
                           url("regular-data/de406-ephemerides/unxp0000.406"),
                           url("regular-data/Earth-orientation-parameters/monthly/bulletinb_IAU2000-216.txt"),
                           url("no-data")).feed(crawler);
        Assert.assertEquals(6, crawler.getCount());
    }

    @Test
    public void compressed() throws OrekitException {
        CountingLoader crawler = new CountingLoader(".*/eopc04.*");
        new NetworkCrawler(url("compressed-data/UTC-TAI.history.gz"),
                           url("compressed-data/eopc04_IAU2000.00.gz"),
                           url("compressed-data/eopc04_IAU2000.02.gz")).feed(crawler);
        Assert.assertEquals(2, crawler.getCount());
    }

    @Test
    public void multiZip() throws OrekitException {
        CountingLoader crawler = new CountingLoader(".*\\.txt$");
        new NetworkCrawler(url("zipped-data/multizip.zip")).feed(crawler);
        Assert.assertEquals(6, crawler.getCount());
    }

    @Test
    public void ioException() throws OrekitException {
        try {
            new NetworkCrawler(url("regular-data/UTC-TAI.history")).feed(new IOExceptionLoader(".*"));
            Assert.fail("an exception should have been thrown");
        } catch (OrekitException oe) {
            // expected behavior
            Assert.assertNotNull(oe.getCause());
            Assert.assertEquals(IOException.class, oe.getCause().getClass());
            Assert.assertEquals("dummy error", oe.getMessage());
        } catch (Exception e) {
            Assert.fail("wrong exception caught");
        }
    }

    @Test
    public void parseException() throws OrekitException {
        try {
            new NetworkCrawler(url("regular-data/UTC-TAI.history")).feed(new ParseExceptionLoader(".*"));
            Assert.fail("an exception should have been thrown");
        } catch (OrekitException oe) {
            // expected behavior
            Assert.assertNotNull(oe.getCause());
            Assert.assertEquals(ParseException.class, oe.getCause().getClass());
            Assert.assertEquals("dummy error", oe.getMessage());
        } catch (Exception e) {
            e.printStackTrace(System.err);
            Assert.fail("wrong exception caught");
        }
    }

    private static class CountingLoader implements DataLoader {
        private Pattern namePattern;
        private int count;
        public CountingLoader(String pattern) {
            namePattern = Pattern.compile(pattern);
            count = 0;
        }
        public void loadData(InputStream input, String name) {
            ++count;
        }
        public int getCount() {
            return count;
        }
        public boolean fileIsSupported(String fileName) {
            return namePattern.matcher(fileName).matches();
        }
    }

    private static class IOExceptionLoader implements DataLoader {
        private Pattern namePattern;
        public IOExceptionLoader(String pattern) {
            namePattern = Pattern.compile(pattern);
        }
        public void loadData(InputStream input, String name) throws IOException {
            if (name.endsWith("UTC-TAI.history")) {
                throw new IOException("dummy error");
            }
        }
        public boolean fileIsSupported(String fileName) {
            return namePattern.matcher(fileName).matches();
        }
    }

    private static class ParseExceptionLoader implements DataLoader {
        private Pattern namePattern;
        public ParseExceptionLoader(String pattern) {
            namePattern = Pattern.compile(pattern);
        }
        public void loadData(InputStream input, String name) throws ParseException {
            if (name.endsWith("UTC-TAI.history")) {
                throw new ParseException("dummy error", 0);
            }
        }
        public boolean fileIsSupported(String fileName) {
            return namePattern.matcher(fileName).matches();
        }
    }

    private URL url(String resource) {
        return DirectoryCrawlerTest.class.getClassLoader().getResource(resource);
    }

}
