/*-
 * ========================LICENSE_START=================================
 * utils
 * %%
 * Copyright (C) 2024 Francesco Rusin
 * %%
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * =========================LICENSE_END==================================
 */
package utils;

import java.util.List;
import java.util.Set;

public class UnorderedPair<T> {
  private final Set<T> set;

  public UnorderedPair(T first, T second) {
    set = Set.of(first, second);
  }

  public List<T> elements() {
    return set.stream().toList();
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof UnorderedPair<?> pair) {
      return set.equals(pair.set);
    }
    return false;
  }

  @Override
  public int hashCode() {
    return set.hashCode();
  }

  @Override
  public String toString() {
    return elements().toString();
  }
}
